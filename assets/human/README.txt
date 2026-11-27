CHARACTER RIG PACK - 360 DEGREE SHOULDER CONTROLS

Files
- Character_Rig.blend: editable Blender project with separate control and deformation armatures.
- Character_Rig.fbx: static skinned model and deformation skeleton. The Blender control rig/IK constraints are not included in FBX.
- Character_Preview.png: neutral grayscale preview.
- Character_Mobility_Test.png: side-view elbow/knee bend test.

Orientation and materials
- Character front faces +Y; the floor arrow and FRONT +Y label identify the front.
- Character materials and viewport shading are white/grayscale.
- Original block-shaped body and limb halves are preserved. Only local elbow/knee gap bridges are present; there is no full-body skin or bulky joint cover.

Shoulder rotation
- New controls: CTRL_Shoulder_R and CTRL_Shoulder_L in Rig_Controls.
- Each control has free 360-degree rotation on all three Euler axes (X/Y/Z), with translation and scale locked.
- Each shoulder pivot is located at the upper-arm root, so the arm rotates around the shoulder socket rather than around the center of the chest.
- Rotate a shoulder control in Pose Mode to swing the arm through a complete loop. The two shoulders are independent.
- For an unconstrained shoulder loop, leave that side's CTRL_Hand control at its rest position. Moving CTRL_Hand engages the existing IK solver and keeps the hand target in place.

Other movement rules
- Elbows retain front-facing flexion only: up to 150 degrees, with the existing pole controls and hinge limit preserved.
- Knees retain backward flexion only: up to 155 degrees.
- Existing hand/foot IK, reach limits, head connection, and head translation lock are retained.

Project state
- Neutral pose; frame range fixed to 1; zero animation actions and zero NLA tracks.
- Rig_Controls is the active Pose Mode armature when the project opens.
- The FBX is a static skinned export; use the BLEND file for the 360-degree shoulder controls and full IK control rig.