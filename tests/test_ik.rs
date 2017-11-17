extern crate nalgebra as na;
extern crate k;

#[cfg(test)]
mod tests {
    use super::*;
    use na::{Vector3, Translation3};
    use k::InverseKinematicsSolver;
    use k::KinematicChain;
    use k::JointContainer;

    pub fn create_joint_with_link_array6() -> Vec<k::Link<f32>> {
        let l0 = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint(
                "shoulder_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .finalize();
        let l1 = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint(
                "shoulder_roll",
                k::JointType::Rotational { axis: Vector3::x_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize();
        let l2 = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint(
                "shoulder_yaw",
                k::JointType::Rotational { axis: Vector3::z_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize();
        let l3 = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint(
                "elbow_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l4 = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint(
                "wrist_yaw",
                k::JointType::Rotational { axis: Vector3::z_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l5 = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint(
                "wrist_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        vec![l0, l1, l2, l3, l4, l5]
    }


    pub fn create_joint_with_link_array7() -> Vec<k::Link<f32>> {
        let l0 = k::LinkBuilder::new()
            .name("shoulder_link1")
            .joint(
                "shoulder_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .finalize();
        let l1 = k::LinkBuilder::new()
            .name("shoulder_link2")
            .joint(
                "shoulder_roll",
                k::JointType::Rotational { axis: Vector3::x_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.1, 0.0))
            .finalize();
        let l2 = k::LinkBuilder::new()
            .name("shoulder_link3")
            .joint(
                "shoulder_yaw",
                k::JointType::Rotational { axis: Vector3::z_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.30))
            .finalize();
        let l3 = k::LinkBuilder::new()
            .name("elbow_link1")
            .joint(
                "elbow_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l4 = k::LinkBuilder::new()
            .name("wrist_link1")
            .joint(
                "wrist_yaw",
                k::JointType::Rotational { axis: Vector3::z_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l5 = k::LinkBuilder::new()
            .name("wrist_link2")
            .joint(
                "wrist_pitch",
                k::JointType::Rotational { axis: Vector3::y_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.15))
            .finalize();
        let l6 = k::LinkBuilder::new()
            .name("wrist_link3")
            .joint(
                "wrist_roll",
                k::JointType::Rotational { axis: Vector3::x_axis() },
                None,
            )
            .translation(Translation3::new(0.0, 0.0, -0.10))
            .finalize();
        vec![l0, l1, l2, l3, l4, l5, l6]
    }

    #[test]
    pub fn ik_fk7() {
        let links = create_joint_with_link_array7();
        let mut tree = k::IdTree::new();
        let id_list = links
            .into_iter()
            .map(|link| tree.create_node(link))
            .collect::<Vec<_>>();
        let mut arm = k::IdKinematicChain::new("arm7", &mut tree, &id_list);

        let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
        arm.set_joint_angles(&angles).unwrap();
        let init_pose = arm.calc_end_transform();
        let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 100);
        solver.solve(&mut arm, &init_pose).unwrap();
        let end_angles = arm.get_joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }

    #[test]
    pub fn ik_fk6() {
        let links = create_joint_with_link_array6();
        let mut tree = k::IdTree::new();
        let id_list = links
            .into_iter()
            .map(|link| tree.create_node(link))
            .collect::<Vec<_>>();
        let mut arm = k::IdKinematicChain::new("arm6", &mut tree, &id_list);

        let angles = vec![0.8, 0.2, 0.0, -1.2, 0.0, 0.1];
        arm.set_joint_angles(&angles).unwrap();
        let init_pose = arm.calc_end_transform();
        let solver = k::JacobianIKSolverBuilder::new().finalize();
        // set different angles
        arm.set_joint_angles(&[0.4, 0.1, 0.1, -1.0, 0.1, 0.1])
            .unwrap();
        solver.solve(&mut arm, &init_pose).unwrap();
        let end_angles = arm.get_joint_angles();
        for (init, end) in angles.iter().zip(end_angles.iter()) {
            assert!((init - end).abs() < 0.001);
        }
    }
}
