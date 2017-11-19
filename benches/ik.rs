// rustup run nightly cargo bench
#![feature(test)]

extern crate test;
extern crate k;
extern crate nalgebra as na;

use k::KinematicChain;
use k::urdf::FromUrdf;
use k::InverseKinematicsSolver;
use k::CreateChain;


fn bench_tree_ik<K>(mut arm: K, b: &mut test::Bencher)
where
    K: KinematicChain<f64>,
{
    // set joint angles
    let angles = vec![0.5, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_angles(&angles).unwrap();
    let mut target = arm.calc_end_transform();
    target.translation.vector[0] += 0.02;

    let solver = k::JacobianIKSolver::new(0.001, 0.001, 0.001, 1000);
    b.iter(|| {
        solver.solve(&mut arm, &target).unwrap();
        let _trans = arm.calc_end_transform();
        arm.set_joint_angles(&angles).unwrap();
    });
}


#[bench]
fn bench_idtree_ik(b: &mut test::Bencher) {
    let mut robot = k::IdLinkTree::<f64>::from_urdf_file::<f64, _>("urdf/sample.urdf").unwrap();
    let arm = robot.chain_from_end_link_name("l_wrist2").unwrap();
    bench_tree_ik(arm, b);
}

#[bench]
fn bench_rctree_ik(b: &mut test::Bencher) {
    let mut robot = k::RcLinkTree::<f64>::from_urdf_file::<f64, _>("urdf/sample.urdf").unwrap();
    let arm = robot.chain_from_end_link_name("l_wrist2").unwrap();
    bench_tree_ik(arm, b);
}
