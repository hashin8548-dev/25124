pub(crate) fn voxel_instance_id(chunk_id: &str, x: u32, y: u32, z: u32) -> String {
    format!("voxel:{chunk_id}:{x}:{y}:{z}")
}

/// Parses a stable voxel ID without assuming that chunk IDs contain no separators.
pub(crate) fn parse_voxel_instance_id(id: &str) -> Option<(&str, [u32; 3])> {
    let mut parts = id.rsplitn(4, ':');
    let z = parts.next()?.parse().ok()?;
    let y = parts.next()?.parse().ok()?;
    let x = parts.next()?.parse().ok()?;
    let chunk_id = parts.next()?.strip_prefix("voxel:")?;
    if chunk_id.is_empty() {
        return None;
    }
    Some((chunk_id, [x, y, z]))
}

#[cfg(test)]
mod tests {
    use super::{parse_voxel_instance_id, voxel_instance_id};

    #[test]
    fn voxel_id_parser_preserves_chunk_ids_with_separators() {
        let id = voxel_instance_id("region_a:chunk_0_0", 1, 2, 3);
        assert_eq!(
            parse_voxel_instance_id(&id),
            Some(("region_a:chunk_0_0", [1, 2, 3]))
        );
        assert_eq!(parse_voxel_instance_id("voxel:chunk:x:y:z"), None);
    }

    #[test]
    fn voxel_ids_are_stable_and_coordinate_specific() {
        assert_eq!(
            voxel_instance_id("chunk_0_0", 1, 2, 3),
            "voxel:chunk_0_0:1:2:3"
        );
        assert_ne!(
            voxel_instance_id("chunk_0_0", 1, 2, 3),
            voxel_instance_id("chunk_0_0", 3, 2, 1)
        );
    }
}
