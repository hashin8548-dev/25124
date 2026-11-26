pub(crate) fn voxel_instance_id(chunk_id: &str, x: u32, y: u32, z: u32) -> String {
    format!("voxel:{chunk_id}:{x}:{y}:{z}")
}

#[cfg(test)]
mod tests {
    use super::voxel_instance_id;

    #[test]
    fn voxel_ids_are_stable_and_coordinate_specific() {
        assert_eq!(voxel_instance_id("chunk_0_0", 1, 2, 3), "voxel:chunk_0_0:1:2:3");
        assert_ne!(
            voxel_instance_id("chunk_0_0", 1, 2, 3),
            voxel_instance_id("chunk_0_0", 3, 2, 1)
        );
    }
}
