use std::collections::{HashMap, VecDeque};

use bevy::prelude::Vec3;

/// A rigid-body node used by the structural load-path approximation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct StructuralNode {
    pub(crate) position_m: Vec3,
    pub(crate) mass_kg: f32,
    pub(crate) applied_force_n: Vec3,
    pub(crate) fixed: bool,
}

/// A square-section axial member connecting two nodes.
#[derive(Clone, Debug)]
pub(crate) struct StructuralEdge {
    pub(crate) id: String,
    pub(crate) node_a: usize,
    pub(crate) node_b: usize,
    pub(crate) elastic_modulus_pa: f32,
    pub(crate) area_m2: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct AxialBondStress {
    pub(crate) tensile_pa: f32,
    pub(crate) compressive_pa: f32,
}

/// Estimates axial stress in an anchored bond network under gravity and applied nodal forces.
///
/// This is a linear-elastic, small-displacement truss approximation. It solves a sparse
/// positive-semidefinite stiffness system with diagonally preconditioned conjugate gradients.
/// A tiny numerical regularizer stabilizes directions not constrained by axial members; it
/// does not replace the full fixed-joint solver for shear, bending, or rotational response.
///
/// Components without a path to a fixed node are intentionally omitted: uniform gravity
/// accelerates a free-falling structure as a whole and does not create self-weight stress.
pub(crate) fn solve_structural_loads(
    nodes: &[StructuralNode],
    edges: &[StructuralEdge],
    gravity_m_s2: Vec3,
) -> HashMap<String, AxialBondStress> {
    let mut stresses = HashMap::new();
    if nodes.is_empty() || edges.is_empty() || !gravity_m_s2.is_finite() {
        return stresses;
    }

    let mut adjacency = vec![Vec::<usize>::new(); nodes.len()];
    for (edge_index, edge) in edges.iter().enumerate() {
        if edge.node_a >= nodes.len()
            || edge.node_b >= nodes.len()
            || edge.node_a == edge.node_b
            || !edge.elastic_modulus_pa.is_finite()
            || edge.elastic_modulus_pa <= 0.0
            || !edge.area_m2.is_finite()
            || edge.area_m2 <= 0.0
        {
            continue;
        }
        adjacency[edge.node_a].push(edge_index);
        adjacency[edge.node_b].push(edge_index);
    }

    let mut anchored = vec![false; nodes.len()];
    let mut queue = VecDeque::new();
    for (index, node) in nodes.iter().enumerate() {
        if node.fixed {
            anchored[index] = true;
            queue.push_back(index);
        }
    }

    while let Some(node_index) = queue.pop_front() {
        for &edge_index in &adjacency[node_index] {
            let edge = &edges[edge_index];
            let other = if edge.node_a == node_index {
                edge.node_b
            } else {
                edge.node_a
            };
            if !anchored[other] {
                anchored[other] = true;
                queue.push_back(other);
            }
        }
    }

    let mut free_index = vec![None; nodes.len()];
    let mut free_nodes = Vec::new();
    for (index, node) in nodes.iter().enumerate() {
        if anchored[index]
            && !node.fixed
            && node.mass_kg.is_finite()
            && node.mass_kg >= 0.0
            && node.applied_force_n.is_finite()
            && node.position_m.is_finite()
            && (node.mass_kg > 0.0 || node.applied_force_n.length_squared() > 0.0)
        {
            free_index[index] = Some(free_nodes.len());
            free_nodes.push(index);
        }
    }
    if free_nodes.is_empty() {
        return stresses;
    }

    let mut prepared_edges = Vec::new();
    for (edge_index, edge) in edges.iter().enumerate() {
        if !anchored.get(edge.node_a).copied().unwrap_or(false)
            || !anchored.get(edge.node_b).copied().unwrap_or(false)
            || !edge.elastic_modulus_pa.is_finite()
            || !edge.area_m2.is_finite()
            || edge.elastic_modulus_pa <= 0.0
            || edge.area_m2 <= 0.0
        {
            continue;
        }
        let delta = nodes[edge.node_b].position_m - nodes[edge.node_a].position_m;
        let length = delta.length();
        if !length.is_finite() || length <= 1.0e-5 {
            continue;
        }
        let stiffness = edge.elastic_modulus_pa * edge.area_m2 / length;
        if !stiffness.is_finite() || stiffness <= 0.0 {
            continue;
        }
        prepared_edges.push(PreparedEdge {
            source_index: edge_index,
            a: free_index[edge.node_a],
            b: free_index[edge.node_b],
            normal: delta / length,
            stiffness,
        });
    }
    if prepared_edges.is_empty() {
        return stresses;
    }

    let mut diagonal = vec![Vec3::ZERO; free_nodes.len()];
    for edge in &prepared_edges {
        let weighted = edge.normal * edge.normal * edge.stiffness;
        if let Some(a) = edge.a {
            diagonal[a] += weighted;
        }
        if let Some(b) = edge.b {
            diagonal[b] += weighted;
        }
    }
    let stiffness_scale = diagonal
        .iter()
        .map(|value| value.max_element())
        .fold(0.0_f32, f32::max);
    if !stiffness_scale.is_finite() || stiffness_scale <= 0.0 {
        return stresses;
    }

    // This is only a numerical stabilization for unsupported transverse modes. Its scale is
    // bounded relative to the actual member stiffness and tests verify load-path equilibrium.
    let regularization = stiffness_scale * 1.0e-8 + 1.0e-8;
    for value in &mut diagonal {
        *value += Vec3::splat(regularization);
    }

    let mut rhs = vec![Vec3::ZERO; free_nodes.len()];
    for (free_slot, &node_index) in free_nodes.iter().enumerate() {
        rhs[free_slot] =
            gravity_m_s2 * nodes[node_index].mass_kg + nodes[node_index].applied_force_n;
    }

    let mut solution = vec![Vec3::ZERO; free_nodes.len()];
    let mut residual = rhs.clone();
    let mut preconditioned: Vec<Vec3> = residual
        .iter()
        .zip(&diagonal)
        .map(|(value, diagonal)| *value / *diagonal)
        .collect();
    let mut direction = preconditioned.clone();
    let mut rz = dot_vectors(&residual, &preconditioned);
    let rhs_norm = dot_vectors(&rhs, &rhs).sqrt().max(1.0);

    for _ in 0..160 {
        if !rz.is_finite() || rz <= 1.0e-24 {
            break;
        }
        let product = apply_stiffness(
            &direction,
            &prepared_edges,
            regularization,
            free_nodes.len(),
        );
        let denominator = dot_vectors(&direction, &product);
        if !denominator.is_finite() || denominator <= 1.0e-24 {
            break;
        }
        let alpha = rz / denominator;
        if !alpha.is_finite() {
            break;
        }
        for index in 0..solution.len() {
            solution[index] += direction[index] * alpha;
            residual[index] -= product[index] * alpha;
        }
        if dot_vectors(&residual, &residual).sqrt() <= rhs_norm * 1.0e-5 {
            break;
        }

        preconditioned = residual
            .iter()
            .zip(&diagonal)
            .map(|(value, diagonal)| *value / *diagonal)
            .collect();
        let next_rz = dot_vectors(&residual, &preconditioned);
        if !next_rz.is_finite() || next_rz < 0.0 {
            break;
        }
        let beta = next_rz / rz;
        for index in 0..direction.len() {
            direction[index] = preconditioned[index] + direction[index] * beta;
        }
        rz = next_rz;
    }

    for prepared in prepared_edges {
        let edge = &edges[prepared.source_index];
        let displacement_a = prepared.a.map_or(Vec3::ZERO, |index| solution[index]);
        let displacement_b = prepared.b.map_or(Vec3::ZERO, |index| solution[index]);
        let extension_m = (displacement_b - displacement_a).dot(prepared.normal);
        let axial_force_n = prepared.stiffness * extension_m;
        let axial_stress_pa = axial_force_n / edge.area_m2;
        if !axial_stress_pa.is_finite() {
            continue;
        }
        stresses.insert(
            edge.id.clone(),
            if axial_stress_pa >= 0.0 {
                AxialBondStress {
                    tensile_pa: axial_stress_pa,
                    compressive_pa: 0.0,
                }
            } else {
                AxialBondStress {
                    tensile_pa: 0.0,
                    compressive_pa: -axial_stress_pa,
                }
            },
        );
    }

    stresses
}

#[derive(Clone, Copy)]
struct PreparedEdge {
    source_index: usize,
    a: Option<usize>,
    b: Option<usize>,
    normal: Vec3,
    stiffness: f32,
}

fn apply_stiffness(
    input: &[Vec3],
    edges: &[PreparedEdge],
    regularization: f32,
    node_count: usize,
) -> Vec<Vec3> {
    let mut output: Vec<Vec3> = input.iter().map(|value| *value * regularization).collect();

    for edge in edges {
        let displacement_a = edge.a.map_or(Vec3::ZERO, |index| input[index]);
        let displacement_b = edge.b.map_or(Vec3::ZERO, |index| input[index]);
        let axial_response =
            edge.normal * ((displacement_b - displacement_a).dot(edge.normal) * edge.stiffness);
        if let Some(a) = edge.a {
            output[a] -= axial_response;
        }
        if let Some(b) = edge.b {
            output[b] += axial_response;
        }
    }

    debug_assert_eq!(output.len(), node_count);
    output
}

fn dot_vectors(left: &[Vec3], right: &[Vec3]) -> f32 {
    left.iter()
        .zip(right)
        .map(|(left, right)| left.dot(*right))
        .sum()
}

#[cfg(test)]
mod tests {
    use super::{StructuralEdge, StructuralNode, solve_structural_loads};
    use bevy::prelude::Vec3;

    fn edge(id: &str, a: usize, b: usize) -> StructuralEdge {
        StructuralEdge {
            id: id.to_owned(),
            node_a: a,
            node_b: b,
            elastic_modulus_pa: 1.0e6,
            area_m2: 1.0,
        }
    }

    #[test]
    fn gravity_on_an_unanchored_structure_does_not_create_internal_stress() {
        let nodes = [
            StructuralNode {
                position_m: Vec3::ZERO,
                mass_kg: 5.0,
                applied_force_n: Vec3::ZERO,
                fixed: false,
            },
            StructuralNode {
                position_m: Vec3::Y,
                mass_kg: 5.0,
                applied_force_n: Vec3::ZERO,
                fixed: false,
            },
        ];
        let result = solve_structural_loads(&nodes, &[edge("floating", 0, 1)], Vec3::NEG_Y * 9.81);
        assert!(result.is_empty());
    }

    #[test]
    fn anchored_column_transmits_self_weight_as_compression() {
        let nodes = [
            StructuralNode {
                position_m: Vec3::ZERO,
                mass_kg: 0.0,
                applied_force_n: Vec3::ZERO,
                fixed: true,
            },
            StructuralNode {
                position_m: Vec3::Y,
                mass_kg: 10.0,
                applied_force_n: Vec3::ZERO,
                fixed: false,
            },
        ];
        let result = solve_structural_loads(&nodes, &[edge("column", 0, 1)], Vec3::NEG_Y * 10.0);
        let stress = result.get("column").unwrap();
        assert_eq!(stress.tensile_pa, 0.0);
        assert!((stress.compressive_pa - 100.0).abs() < 0.01);
    }

    #[test]
    fn contact_load_propagates_through_an_anchored_serial_structure() {
        let nodes = [
            StructuralNode {
                position_m: Vec3::ZERO,
                mass_kg: 0.0,
                applied_force_n: Vec3::ZERO,
                fixed: true,
            },
            StructuralNode {
                position_m: Vec3::Y,
                mass_kg: 0.0,
                applied_force_n: Vec3::NEG_Y * 100.0,
                fixed: false,
            },
        ];
        let result = solve_structural_loads(&nodes, &[edge("impact-path", 0, 1)], Vec3::ZERO);
        let stress = result.get("impact-path").unwrap();
        assert_eq!(stress.tensile_pa, 0.0);
        assert!((stress.compressive_pa - 100.0).abs() < 0.1);
    }
    #[test]
    fn parallel_load_paths_share_gravity_in_proportion_to_member_stiffness() {
        let nodes = [
            StructuralNode {
                position_m: Vec3::ZERO,
                mass_kg: 0.0,
                applied_force_n: Vec3::ZERO,
                fixed: true,
            },
            StructuralNode {
                position_m: Vec3::Y,
                mass_kg: 10.0,
                applied_force_n: Vec3::ZERO,
                fixed: false,
            },
        ];
        let edges = [edge("left", 0, 1), edge("right", 0, 1)];
        let result = solve_structural_loads(&nodes, &edges, Vec3::NEG_Y * 10.0);
        let left = result.get("left").unwrap().compressive_pa;
        let right = result.get("right").unwrap().compressive_pa;
        assert!((left - right).abs() < 0.01);
        assert!((left - 50.0).abs() < 0.1);
    }

    #[test]
    fn removing_a_failed_parallel_member_redistributes_load_to_survivor() {
        let nodes = [
            StructuralNode {
                position_m: Vec3::ZERO,
                mass_kg: 0.0,
                applied_force_n: Vec3::ZERO,
                fixed: true,
            },
            StructuralNode {
                position_m: Vec3::Y,
                mass_kg: 10.0,
                applied_force_n: Vec3::ZERO,
                fixed: false,
            },
        ];
        let before = [edge("left", 0, 1), edge("right", 0, 1)];
        let after = [edge("right", 0, 1)];
        let before_loads = solve_structural_loads(&nodes, &before, Vec3::NEG_Y * 10.0);
        let after_loads = solve_structural_loads(&nodes, &after, Vec3::NEG_Y * 10.0);
        assert!((after_loads["right"].compressive_pa - 100.0).abs() < 0.1);
        assert!(after_loads["right"].compressive_pa > before_loads["right"].compressive_pa * 1.8);
    }
}
