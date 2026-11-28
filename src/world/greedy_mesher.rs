use std::collections::BTreeMap;

use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, Mesh, PrimitiveTopology, VertexAttributeValues},
    prelude::*,
};

use crate::config::MAX_CHUNK_DIMENSION;

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
        if dimensions.contains(&0)
            || dimensions
                .iter()
                .any(|dimension| *dimension > MAX_CHUNK_DIMENSION as usize)
        {
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
        self.build_greedy_meshes_internal(renderable, false)
    }

    /// Builds an adaptive surface LOD while retaining exact side walls, material boundaries,
    /// chunk-edge samples, and all non-topmost horizontal surfaces. Only the exposed upper
    /// terrain surface is allowed to approximate within the caller's world-space error bound.
    pub fn build_surface_lod_meshes(
        &self,
        renderable: &[bool],
        max_error_m: f32,
    ) -> Result<BTreeMap<u8, Mesh>, String> {
        if !max_error_m.is_finite() || max_error_m < 0.0 {
            return Err("LOD surface error must be finite and non-negative".to_owned());
        }
        let mut meshes = self.build_greedy_meshes_internal(renderable, true)?;
        let [width, _, depth] = self.dimensions;
        let mut top_samples = vec![None; width * depth];
        let mut palettes = std::collections::BTreeSet::new();

        for z in 0..depth {
            for x in 0..width {
                for y in (0..self.dimensions[1]).rev() {
                    let index = self.index(x, y, z);
                    let palette = self.cells[index];
                    if palette != 0 && renderable[index] {
                        let sample = SurfaceSample {
                            palette,
                            height_m: (y + 1) as f32 * self.voxel_size_m,
                        };
                        top_samples[z * width + x] = Some(sample);
                        palettes.insert(palette);
                        break;
                    }
                }
            }
        }

        for palette in palettes {
            let mut buffers = MeshBuffers::default();

            let mut context = AdaptiveSurfaceContext {
                samples: &top_samples,
                width,
                depth,
                palette,
                voxel_size_m: self.voxel_size_m,
                max_error_m,
                buffers: &mut buffers,
            };
            emit_adaptive_surface_region(&mut context, 0, width, 0, depth);

            if buffers.indices.is_empty() {
                continue;
            }
            let top_mesh = mesh_from_buffers(buffers);
            if let Some(existing) = meshes.get_mut(&palette) {
                append_mesh(existing, &top_mesh)?;
            } else {
                meshes.insert(palette, top_mesh);
            }
        }

        Ok(meshes)
    }

    fn build_greedy_meshes_internal(
        &self,
        renderable: &[bool],
        suppress_topmost_surfaces: bool,
    ) -> Result<BTreeMap<u8, Mesh>, String> {
        if renderable.len() != self.cells.len() {
            return Err("renderable mask does not match the voxel grid volume".to_owned());
        }
        if renderable
            .iter()
            .zip(&self.cells)
            .any(|(is_renderable, palette)| *is_renderable && *palette == 0)
        {
            return Err("air cells cannot be marked as renderable".to_owned());
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
                                self.cells[adjacent_index] != 0 && renderable[adjacent_index]
                            };
                            let omit_adaptive_top = suppress_topmost_surfaces
                                && axis == 1
                                && sign > 0
                                && !self
                                    .has_renderable_above(cell[0], cell[1], cell[2], renderable);
                            if !adjacent_is_mergeable && !omit_adaptive_top {
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

    /// Builds a factor-two, visual-only approximation from mergeable source cells.
    /// Decomposes equal-material occupied cells into maximal axis-aligned boxes. Each box
    /// becomes one child shape in the chunk's compound collider.
    pub fn greedy_boxes(&self, mergeable: &[bool]) -> Result<Vec<GreedyBox>, String> {
        if mergeable.len() != self.cells.len() {
            return Err("mergeable mask does not match the voxel grid volume".to_owned());
        }
        if mergeable
            .iter()
            .zip(&self.cells)
            .any(|(is_mergeable, palette)| *is_mergeable && *palette == 0)
        {
            return Err("air cells cannot be marked as mergeable".to_owned());
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
                                if visited[next] || !mergeable[next] || self.cells[next] != palette
                                {
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

    fn has_renderable_above(&self, x: usize, y: usize, z: usize, renderable: &[bool]) -> bool {
        (y + 1..self.dimensions[1]).any(|above_y| renderable[self.index(x, above_y, z)])
    }

    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        (y * self.dimensions[2] + z) * self.dimensions[0] + x
    }
}

#[derive(Clone, Copy)]
struct SurfaceSample {
    palette: u8,
    height_m: f32,
}

struct AdaptiveSurfaceContext<'a> {
    samples: &'a [Option<SurfaceSample>],
    width: usize,
    depth: usize,
    palette: u8,
    voxel_size_m: f32,
    max_error_m: f32,
    buffers: &'a mut MeshBuffers,
}

fn emit_adaptive_surface_region(
    context: &mut AdaptiveSurfaceContext<'_>,
    x0: usize,
    x1: usize,
    z0: usize,
    z1: usize,
) {
    if x0 >= x1 || z0 >= z1 {
        return;
    }

    let mut covered = true;
    for z in z0..z1 {
        for x in x0..x1 {
            if !context.samples[z * context.width + x]
                .is_some_and(|sample| sample.palette == context.palette)
            {
                covered = false;
                break;
            }
        }
        if !covered {
            break;
        }
    }

    if covered {
        let h00 = context.samples[z0 * context.width + x0].unwrap().height_m;
        let h10 = context.samples[z0 * context.width + x1 - 1]
            .unwrap()
            .height_m;
        let h01 = context.samples[(z1 - 1) * context.width + x0]
            .unwrap()
            .height_m;
        let h11 = context.samples[(z1 - 1) * context.width + x1 - 1]
            .unwrap()
            .height_m;
        let span_x = (x1 - x0) as f32;
        let span_z = (z1 - z0) as f32;
        let mut max_error = 0.0_f32;
        let mut min_height = f32::INFINITY;
        let mut max_height = f32::NEG_INFINITY;

        for z in z0..z1 {
            let tz = (z - z0) as f32 / span_z + 0.5 / span_z;
            for x in x0..x1 {
                let tx = (x - x0) as f32 / span_x + 0.5 / span_x;
                let lower = h00 + (h10 - h00) * tx;
                let upper = h01 + (h11 - h01) * tx;
                let predicted = lower + (upper - lower) * tz;
                let actual = context.samples[z * context.width + x].unwrap().height_m;
                min_height = min_height.min(actual);
                max_height = max_height.max(actual);
                max_error = max_error.max((predicted - actual).abs());
            }
        }

        // The error bound is measured at every source cell centre. Patches share deterministic
        // edge samples; deviations are bounded and no generated point leaves the source bounds.
        let touches_chunk_boundary =
            x0 == 0 || z0 == 0 || x1 == context.width || z1 == context.depth;
        let boundary_is_exact = !touches_chunk_boundary || max_height - min_height <= 1.0e-6;
        if max_error <= context.max_error_m && boundary_is_exact {
            append_surface_quad(
                context.buffers,
                x0 as f32 * context.voxel_size_m,
                x1 as f32 * context.voxel_size_m,
                z0 as f32 * context.voxel_size_m,
                z1 as f32 * context.voxel_size_m,
                [h00, h10, h11, h01],
            );
            return;
        }
    }

    let span_x = x1 - x0;
    let span_z = z1 - z0;
    if span_x == 1 && span_z == 1 {
        if let Some(sample) = context.samples[z0 * context.width + x0]
            && sample.palette == context.palette
        {
            append_surface_quad(
                context.buffers,
                x0 as f32 * context.voxel_size_m,
                x1 as f32 * context.voxel_size_m,
                z0 as f32 * context.voxel_size_m,
                z1 as f32 * context.voxel_size_m,
                [sample.height_m; 4],
            );
        }
        return;
    }

    let x_mid = x0 + span_x / 2;
    let z_mid = z0 + span_z / 2;
    if span_x > 1 && span_z > 1 {
        emit_adaptive_surface_region(context, x0, x_mid, z0, z_mid);
        emit_adaptive_surface_region(context, x_mid, x1, z0, z_mid);
        emit_adaptive_surface_region(context, x0, x_mid, z_mid, z1);
        emit_adaptive_surface_region(context, x_mid, x1, z_mid, z1);
    } else if span_x > 1 {
        emit_adaptive_surface_region(context, x0, x_mid, z0, z1);
        emit_adaptive_surface_region(context, x_mid, x1, z0, z1);
    } else {
        emit_adaptive_surface_region(context, x0, x1, z0, z_mid);
        emit_adaptive_surface_region(context, x0, x1, z_mid, z1);
    }
}

fn append_surface_quad(
    buffers: &mut MeshBuffers,
    x0: f32,
    x1: f32,
    z0: f32,
    z1: f32,
    heights: [f32; 4],
) {
    let points = [
        Vec3::new(x0, heights[0], z0),
        Vec3::new(x1, heights[1], z0),
        Vec3::new(x1, heights[2], z1),
        Vec3::new(x0, heights[3], z1),
    ];
    for triangle in [[0_usize, 3, 2], [0, 2, 1]] {
        let mut vertices = triangle.map(|index| points[index]);
        let mut normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        if normal.length_squared() <= f32::EPSILON {
            continue;
        }
        normal = normal.normalize();
        if normal.y < 0.0 {
            vertices.swap(1, 2);
            normal = -normal;
        }
        let base = buffers.positions.len() as u32;
        for vertex in vertices {
            buffers.positions.push(vertex.to_array());
            buffers.normals.push(normal.to_array());
            buffers.uvs.push([vertex.x - x0, vertex.z - z0]);
        }
        buffers.indices.extend([base, base + 1, base + 2]);
    }
}

fn mesh_from_buffers(buffers: MeshBuffers) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, buffers.positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, buffers.normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, buffers.uvs);
    mesh.insert_indices(Indices::U32(buffers.indices));
    mesh
}

fn append_mesh(destination: &mut Mesh, source: &Mesh) -> Result<(), String> {
    let Some(VertexAttributeValues::Float32x3(source_positions)) =
        source.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        return Err("LOD source mesh has no float3 position attribute".to_owned());
    };
    let Some(VertexAttributeValues::Float32x3(source_normals)) =
        source.attribute(Mesh::ATTRIBUTE_NORMAL)
    else {
        return Err("LOD source mesh has no float3 normal attribute".to_owned());
    };
    let Some(VertexAttributeValues::Float32x2(source_uvs)) = source.attribute(Mesh::ATTRIBUTE_UV_0)
    else {
        return Err("LOD source mesh has no float2 UV attribute".to_owned());
    };
    let Some(source_indices) = source.indices() else {
        return Err("LOD source mesh has no index buffer".to_owned());
    };

    let Some(VertexAttributeValues::Float32x3(mut positions)) =
        destination.attribute(Mesh::ATTRIBUTE_POSITION).cloned()
    else {
        return Err("LOD destination mesh has no float3 position attribute".to_owned());
    };
    let Some(VertexAttributeValues::Float32x3(mut normals)) =
        destination.attribute(Mesh::ATTRIBUTE_NORMAL).cloned()
    else {
        return Err("LOD destination mesh has no float3 normal attribute".to_owned());
    };
    let Some(VertexAttributeValues::Float32x2(mut uvs)) =
        destination.attribute(Mesh::ATTRIBUTE_UV_0).cloned()
    else {
        return Err("LOD destination mesh has no float2 UV attribute".to_owned());
    };
    let Some(Indices::U32(mut indices)) = destination.indices().cloned() else {
        return Err("LOD destination mesh does not use u32 indices".to_owned());
    };

    let base = positions.len() as u32;
    positions.extend_from_slice(source_positions);
    normals.extend_from_slice(source_normals);
    uvs.extend_from_slice(source_uvs);
    match source_indices {
        Indices::U16(source) => indices.extend(source.iter().map(|index| base + u32::from(*index))),
        Indices::U32(source) => indices.extend(source.iter().map(|index| base + *index)),
    }

    destination.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    destination.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    destination.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    destination.insert_indices(Indices::U32(indices));
    Ok(())
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
    use bevy::mesh::{Mesh, VertexAttributeValues};

    #[test]
    fn two_adjacent_voxels_merge_shared_faces_into_one_mesh_surface() {
        let mut cells = vec![0_u8; 2];
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
    fn greedy_collider_boxes_cover_each_mergeable_cell_once() {
        let cells = vec![1, 1, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0];
        let grid = VoxelGrid::from_cells([3, 2, 2], 0.5, cells).unwrap();
        let mergeable = [
            true, true, true, true, false, false, false, false, false, true, false, false,
        ];
        let boxes = grid.greedy_boxes(&mergeable).unwrap();
        let total_volume = boxes
            .iter()
            .map(|box_| box_.full_size_m.x * box_.full_size_m.y * box_.full_size_m.z)
            .sum::<f32>();
        assert!((total_volume - 5.0 * 0.5_f32.powi(3)).abs() < 1.0e-6);
        assert_eq!(
            boxes.iter().filter(|box_| box_.palette_index == 2).count(),
            1
        );
    }

    #[test]
    fn non_mergeable_voxel_above_is_not_absorbed_into_a_compound_box() {
        let grid = VoxelGrid::from_cells([1, 2, 1], 1.0, vec![1, 1]).unwrap();
        let boxes = grid.greedy_boxes(&[true, false]).unwrap();

        assert_eq!(boxes.len(), 1);
        assert_eq!(boxes[0].center_m, bevy::prelude::Vec3::new(0.5, 0.5, 0.5));
        assert_eq!(boxes[0].full_size_m, bevy::prelude::Vec3::ONE);
    }

    #[test]
    fn mismatched_render_mask_is_rejected() {
        let grid = VoxelGrid::from_cells([1, 1, 1], 1.0, vec![1]).unwrap();
        assert!(grid.build_greedy_meshes(&[]).is_err());
        assert!(grid.greedy_boxes(&[]).is_err());
    }

    #[test]
    fn adaptive_lod_keeps_flat_surface_bounds_and_elevation_exact() {
        let grid = VoxelGrid::from_cells([8, 1, 8], 1.0, vec![1; 64]).unwrap();
        let meshes = grid.build_surface_lod_meshes(&[true; 64], 4.0).unwrap();
        let mesh = meshes.get(&1).unwrap();
        let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() {
            VertexAttributeValues::Float32x3(positions) => positions,
            other => panic!("unexpected position attribute: {other:?}"),
        };
        let normals = match mesh.attribute(Mesh::ATTRIBUTE_NORMAL).unwrap() {
            VertexAttributeValues::Float32x3(normals) => normals,
            other => panic!("unexpected normal attribute: {other:?}"),
        };
        assert_eq!(positions.len(), normals.len());
        let top_surface_vertices = positions
            .iter()
            .zip(normals)
            .filter(|(_, normal)| normal[1] > 0.999);
        let top_surface_vertices = top_surface_vertices.collect::<Vec<_>>();
        assert!(!top_surface_vertices.is_empty());
        assert!(
            top_surface_vertices
                .iter()
                .all(|(position, _)| (position[1] - 1.0).abs() < 1.0e-6)
        );
        let min_x = positions.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min);
        let max_x = positions
            .iter()
            .map(|p| p[0])
            .fold(f32::NEG_INFINITY, f32::max);
        let min_z = positions.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min);
        let max_z = positions
            .iter()
            .map(|p| p[2])
            .fold(f32::NEG_INFINITY, f32::max);
        assert_eq!((min_x, max_x, min_z, max_z), (0.0, 8.0, 0.0, 8.0));
        assert!(
            positions
                .iter()
                .filter(|p| p[1] == 1.0)
                .all(|p| (p[1] - 1.0).abs() < 1.0e-6)
        );
    }

    #[test]
    fn adaptive_lod_uses_error_tolerance_to_control_surface_density() {
        let mut cells = vec![0_u8; 16 * 8 * 16];
        for z in 0..16 {
            for x in 0..16 {
                let height = 1 + ((x + z) / 4).min(7);
                for y in 0..height {
                    cells[(y * 16 + z) * 16 + x] = 1;
                }
            }
        }
        let renderable = cells
            .iter()
            .map(|palette| *palette != 0)
            .collect::<Vec<_>>();
        let grid = VoxelGrid::from_cells([16, 8, 16], 1.0, cells).unwrap();
        let quality = grid.build_surface_lod_meshes(&renderable, 0.05).unwrap();
        let performance = grid.build_surface_lod_meshes(&renderable, 2.0).unwrap();
        let quality_triangles = quality.get(&1).unwrap().indices().unwrap().len();
        let performance_triangles = performance.get(&1).unwrap().indices().unwrap().len();
        assert!(performance_triangles <= quality_triangles);
    }

    #[test]
    fn adaptive_lod_retains_separate_material_surfaces() {
        let grid = VoxelGrid::from_cells([4, 1, 2], 0.5, vec![1, 1, 2, 2, 1, 1, 2, 2]).unwrap();
        let meshes = grid.build_surface_lod_meshes(&[true; 8], 1.0).unwrap();
        assert!(meshes.contains_key(&1));
        assert!(meshes.contains_key(&2));
    }

    #[test]
    fn adaptive_lod_rejects_non_finite_error_tolerance() {
        let grid = VoxelGrid::from_cells([1, 1, 1], 1.0, vec![1]).unwrap();
        assert!(grid.build_surface_lod_meshes(&[true], f32::NAN).is_err());
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
