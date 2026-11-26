use std::collections::BTreeMap;

use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, Mesh, PrimitiveTopology},
    prelude::*,
};

/// Dense, chunk-local palette grid. Index zero is air; non-zero indices are occupied cells.
#[derive(Clone, Debug)]
pub struct VoxelGrid {
    pub dimensions: [usize; 3],
    pub voxel_size_m: f32,
    cells: Vec<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GreedyBox {
    pub palette_index: u8,
    pub center_m: Vec3,
    pub full_size_m: Vec3,
}

#[derive(Default)]
struct MeshBuffers {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    uvs: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl VoxelGrid {
    pub fn from_cells(
        dimensions: [usize; 3],
        voxel_size_m: f32,
        cells: Vec<u8>,
    ) -> Result<Self, String> {
        let expected = dimensions
            .iter()
            .try_fold(1_usize, |volume, dimension| volume.checked_mul(*dimension))
            .ok_or_else(|| "voxel dimensions overflow addressable memory".to_owned())?;
        if dimensions.contains(&0) || dimensions.iter().any(|dimension| *dimension > 32) {
            return Err("voxel dimensions must each be in [1, 32]".to_owned());
        }
        if !voxel_size_m.is_finite() || voxel_size_m <= 0.0 {
            return Err("voxel size must be positive and finite".to_owned());
        }
        if cells.len() != expected {
            return Err(format!(
                "voxel cell count {} does not match grid volume {expected}",
                cells.len()
            ));
        }
        Ok(Self {
            dimensions,
            voxel_size_m,
            cells,
        })
    }

    pub fn cell(&self, x: usize, y: usize, z: usize) -> u8 {
        if x >= self.dimensions[0] || y >= self.dimensions[1] || z >= self.dimensions[2] {
            return 0;
        }
        self.cells[(y * self.dimensions[2] + z) * self.dimensions[0] + x]
    }

    /// Greedily merges coplanar exposed faces, grouped by palette index.
    ///
    /// Only faces between two renderable cells are culled. Interfaces with dynamic or
    /// otherwise non-mergeable cells remain visible, while same-batch internal faces disappear.
    pub fn build_greedy_meshes(&self, renderable: &[bool]) -> Result<BTreeMap<u8, Mesh>, String> {
        if renderable.len() != self.cells.len() {
            return Err("renderable mask does not match the voxel grid volume".to_owned());
        }
        let mut buffers = BTreeMap::<u8, MeshBuffers>::new();
        for axis in 0..3 {
            let u = (axis + 1) % 3;
            let v = (axis + 2) % 3;
            for sign in [-1_i32, 1_i32] {
                for slice in 0..=self.dimensions[axis] {
                    let mut mask = vec![0_u8; self.dimensions[u] * self.dimensions[v]];
                    for v_index in 0..self.dimensions[v] {
                        for u_index in 0..self.dimensions[u] {
                            let cell_axis = if sign > 0 {
                                slice as isize - 1
                            } else {
                                slice as isize
                            };
                            let neighbour_axis = if sign > 0 {
                                slice as isize
                            } else {
                                slice as isize - 1
                            };
                            if cell_axis < 0 || cell_axis >= self.dimensions[axis] as isize {
                                continue;
                            }
                            let mut cell = [0_usize; 3];
                            cell[axis] = cell_axis as usize;
                            cell[u] = u_index;
                            cell[v] = v_index;
                            let mut neighbour = cell;
                            if neighbour_axis >= 0
                                && neighbour_axis < self.dimensions[axis] as isize
                            {
                                neighbour[axis] = neighbour_axis as usize;
                            } else {
                                neighbour[axis] = self.dimensions[axis];
                            }
                            let cell_index = self.index(cell[0], cell[1], cell[2]);
                            let palette = self.cells[cell_index];
                            if palette == 0 || !renderable[cell_index] {
                                continue;
                            }
                            let adjacent_is_mergeable = if neighbour_axis < 0
                                || neighbour_axis >= self.dimensions[axis] as isize
                            {
                                false
                            } else {
                                let adjacent_index =
                                    self.index(neighbour[0], neighbour[1], neighbour[2]);
                                renderable[adjacent_index]
                            };
                            if !adjacent_is_mergeable {
                                mask[v_index * self.dimensions[u] + u_index] = palette;
                            }
                        }
                    }
                    greedy_mask_rectangles(
                        &mut mask,
                        self.dimensions[u],
                        self.dimensions[v],
                        |palette, u_start, v_start, width, height| {
                            let mut origin = [0.0_f32; 3];
                            let mut size = [0.0_f32; 3];
                            // Tiny inward bias avoids coplanar z-fighting at chunk seams and
                            // against independently simulated blocks. Collision geometry remains exact.
                            origin[axis] = slice as f32 * self.voxel_size_m
                                - sign as f32 * self.voxel_size_m * 1.0e-4;
                            origin[u] = u_start as f32 * self.voxel_size_m;
                            origin[v] = v_start as f32 * self.voxel_size_m;
                            size[axis] = 0.0;
                            size[u] = width as f32 * self.voxel_size_m;
                            size[v] = height as f32 * self.voxel_size_m;
                            append_face(
                                buffers.entry(palette).or_default(),
                                origin,
                                size,
                                axis,
                                sign as f32,
                            );
                        },
                    );
                }
            }
        }

        Ok(buffers
            .into_iter()
            .filter_map(|(palette, buffers)| {
                if buffers.indices.is_empty() {
                    return None;
                }
                let mut mesh = Mesh::new(
                    PrimitiveTopology::TriangleList,
                    RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
                );
                mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, buffers.positions);
                mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, buffers.normals);
                mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, buffers.uvs);
                mesh.insert_indices(Indices::U32(buffers.indices));
                Some((palette, mesh))
            })
            .collect())
    }

    /// Decomposes equal-material occupied cells into maximal axis-aligned boxes. Each box
    /// becomes one child shape in the chunk's compound collider.
    pub fn greedy_boxes(&self, mergeable: &[bool]) -> Result<Vec<GreedyBox>, String> {
        if mergeable.len() != self.cells.len() {
            return Err("mergeable mask does not match the voxel grid volume".to_owned());
        }
        let mut visited = vec![false; self.cells.len()];
        let mut boxes = Vec::new();
        let [width, height, depth] = self.dimensions;

        for y in 0..height {
            for z in 0..depth {
                for x in 0..width {
                    let index = self.index(x, y, z);
                    let palette = self.cells[index];
                    if palette == 0 || !mergeable[index] || visited[index] {
                        continue;
                    }

                    let mut size_x = 1;
                    while x + size_x < width {
                        let next = self.index(x + size_x, y, z);
                        if visited[next] || !mergeable[next] || self.cells[next] != palette {
                            break;
                        }
                        size_x += 1;
                    }

                    let mut size_z = 1;
                    'extend_z: while z + size_z < depth {
                        for dx in 0..size_x {
                            let next = self.index(x + dx, y, z + size_z);
                            if visited[next] || !mergeable[next] || self.cells[next] != palette {
                                break 'extend_z;
                            }
                        }
                        size_z += 1;
                    }

                    let mut size_y = 1;
                    'extend_y: while y + size_y < height {
                        for dz in 0..size_z {
                            for dx in 0..size_x {
                                let next = self.index(x + dx, y + size_y, z + dz);
                                if visited[next] || self.cells[next] != palette {
                                    break 'extend_y;
                                }
                            }
                        }
                        size_y += 1;
                    }

                    for dy in 0..size_y {
                        for dz in 0..size_z {
                            for dx in 0..size_x {
                                visited[self.index(x + dx, y + dy, z + dz)] = true;
                            }
                        }
                    }

                    let unit = self.voxel_size_m;
                    boxes.push(GreedyBox {
                        palette_index: palette,
                        center_m: Vec3::new(
                            (x as f32 + size_x as f32 * 0.5) * unit,
                            (y as f32 + size_y as f32 * 0.5) * unit,
                            (z as f32 + size_z as f32 * 0.5) * unit,
                        ),
                        full_size_m: Vec3::new(
                            size_x as f32 * unit,
                            size_y as f32 * unit,
                            size_z as f32 * unit,
                        ),
                    });
                }
            }
        }
        Ok(boxes)
    }

    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        (y * self.dimensions[2] + z) * self.dimensions[0] + x
    }
}

fn greedy_mask_rectangles(
    mask: &mut [u8],
    width: usize,
    height: usize,
    mut emit: impl FnMut(u8, usize, usize, usize, usize),
) {
    for v in 0..height {
        for u in 0..width {
            let palette = mask[v * width + u];
            if palette == 0 {
                continue;
            }
            let mut rect_width = 1;
            while u + rect_width < width && mask[v * width + u + rect_width] == palette {
                rect_width += 1;
            }
            let mut rect_height = 1;
            'height: while v + rect_height < height {
                for du in 0..rect_width {
                    if mask[(v + rect_height) * width + u + du] != palette {
                        break 'height;
                    }
                }
                rect_height += 1;
            }
            for dv in 0..rect_height {
                for du in 0..rect_width {
                    mask[(v + dv) * width + u + du] = 0;
                }
            }
            emit(palette, u, v, rect_width, rect_height);
        }
    }
}

fn append_face(
    buffers: &mut MeshBuffers,
    origin: [f32; 3],
    size: [f32; 3],
    axis: usize,
    sign: f32,
) {
    let u = (axis + 1) % 3;
    let v = (axis + 2) % 3;
    let mut points = [[0.0_f32; 3]; 4];
    points[0] = origin;
    points[1] = origin;
    points[2] = origin;
    points[3] = origin;
    points[1][u] += size[u];
    points[2][u] += size[u];
    points[2][v] += size[v];
    points[3][v] += size[v];

    let normal_axis = axis;
    let mut normal = [0.0_f32; 3];
    normal[normal_axis] = sign;
    let base = buffers.positions.len() as u32;
    buffers.positions.extend(points);
    buffers.normals.extend([normal; 4]);
    buffers.uvs.extend([
        [0.0, 0.0],
        [size[u], 0.0],
        [size[u], size[v]],
        [0.0, size[v]],
    ]);
    if sign > 0.0 {
        buffers
            .indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    } else {
        buffers
            .indices
            .extend([base, base + 2, base + 1, base, base + 3, base + 2]);
    }
}

#[cfg(test)]
mod tests {
    use super::VoxelGrid;

    #[test]
    fn two_adjacent_voxels_merge_shared_faces_into_one_mesh_surface() {
        let mut cells = vec![0_u8; 2 * 1 * 1];
        cells[0] = 1;
        cells[1] = 1;
        let grid = VoxelGrid::from_cells([2, 1, 1], 1.0, cells).unwrap();
        let meshes = grid.build_greedy_meshes(&[true, true]).unwrap();
        let mesh = meshes.get(&1).unwrap();
        assert_eq!(mesh.count_vertices(), 24);
        assert_eq!(mesh.indices().unwrap().len(), 36);
    }

    #[test]
    fn single_voxel_exposes_six_quads() {
        let grid = VoxelGrid::from_cells([1, 1, 1], 1.0, vec![1]).unwrap();
        let meshes = grid.build_greedy_meshes(&[true]).unwrap();
        let mesh = meshes.get(&1).unwrap();
        assert_eq!(mesh.count_vertices(), 24);
        assert_eq!(mesh.indices().unwrap().len(), 36);
    }

    #[test]
    fn non_mergeable_neighbour_keeps_the_interface_face_visible() {
        let grid = VoxelGrid::from_cells([2, 1, 1], 1.0, vec![1, 2]).unwrap();
        let meshes = grid.build_greedy_meshes(&[true, false]).unwrap();
        // The first voxel has five outer faces plus its interface with an interactive cell.
        assert_eq!(meshes.get(&1).unwrap().count_vertices(), 24);
    }

    #[test]
    fn mismatched_render_mask_is_rejected() {
        let grid = VoxelGrid::from_cells([1, 1, 1], 1.0, vec![1]).unwrap();
        assert!(grid.build_greedy_meshes(&[]).is_err());
        assert!(grid.greedy_boxes(&[]).is_err());
    }

    #[test]
    fn homogeneous_static_slab_becomes_one_compound_shape_box() {
        let grid = VoxelGrid::from_cells([16, 1, 16], 1.0, vec![1; 16 * 16]).unwrap();
        let boxes = grid.greedy_boxes(&vec![true; 16 * 16]).unwrap();
        assert_eq!(boxes.len(), 1);
        assert_eq!(
            boxes[0].full_size_m,
            bevy::prelude::Vec3::new(16.0, 1.0, 16.0)
        );
    }

    #[test]
    fn disconnected_materials_are_never_merged_into_the_same_collider_box() {
        let grid = VoxelGrid::from_cells([3, 1, 1], 0.5, vec![1, 2, 1]).unwrap();
        let boxes = grid.greedy_boxes(&[true, true, true]).unwrap();
        assert_eq!(boxes.len(), 3);
        assert_eq!(boxes[0].palette_index, 1);
        assert_eq!(boxes[1].palette_index, 2);
    }
}
