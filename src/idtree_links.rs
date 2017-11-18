/*
   Copyright 2017 Takashi Ogura

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */
extern crate nalgebra as na;

use na::{Isometry3, Real};
use std::slice::{Iter, IterMut};

use errors::*;
use joints::*;
use traits::*;
use links::*;
use idtree::*;

pub type IdLink<T> = IdNode<Link<T>>;

/// Kinematic chain using `IdNode<Link<T>>`
pub struct IdKinematicChain<'a, T: Real> {
    pub name: String,
    pub id_list: Vec<NodeId>,
    pub transform: Isometry3<T>,
    pub tree: &'a mut IdTree<Link<T>>,
    end_link_name: Option<String>,
}

impl<'a, T> IdKinematicChain<'a, T>
where
    T: Real,
{
    pub fn new(name: &str, tree: &'a mut IdTree<Link<T>>, id_list: &[NodeId]) -> Self {
        IdKinematicChain {
            name: name.to_string(),
            id_list: id_list.to_vec(),
            tree,
            transform: Isometry3::identity(),
            end_link_name: None,
        }
    }
}

impl<'a, T> KinematicChain<T> for IdKinematicChain<'a, T>
where
    T: Real,
{
    fn calc_end_transform(&self) -> Isometry3<T> {
        let mut end_transform = self.transform.clone();
        // TODO: use fold
        for id in &self.id_list {
            end_transform *= self.tree.get(id).data.calc_transform();
            if let Some(ref end_name) = self.end_link_name {
                if end_name.to_owned() == self.tree.get(id).data.name {
                    return end_transform;
                }
            }
        }
        end_transform
    }
}

impl<'a, T> LinkContainer<T> for IdKinematicChain<'a, T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.id_list
            .iter()
            .scan(self.transform, |base, id| {
                *base *= self.tree.get(id).data.calc_transform();
                Some(*base)
            })
            .collect()
    }
    fn get_link_names(&self) -> Vec<String> {
        self.id_list
            .iter()
            .map(|id| self.tree.get(id).data.name.to_owned())
            .collect()
    }
}

impl<'a, T> JointContainer<T> for IdKinematicChain<'a, T>
where
    T: Real,
{
    fn set_joint_angles(&mut self, angles: &[T]) -> Result<(), JointError> {
        // TODO: is it possible to cache the joint_with_angle to speed up?
        let links_with_angle = self.id_list
            .iter()
            .filter(|id| self.tree.get(id).data.has_joint_angle())
            .collect::<Vec<_>>();
        if links_with_angle.len() != angles.len() {
            println!("angles={:?}", angles);
            return Err(JointError::SizeMisMatch);
        }
        for (i, id) in links_with_angle.into_iter().enumerate() {
            try!(self.tree.get_mut(id).data.set_joint_angle(angles[i]));
        }
        Ok(())
    }
    fn get_joint_angles(&self) -> Vec<T> {
        self.id_list
            .iter()
            .filter_map(|id| self.tree.get(id).data.get_joint_angle())
            .collect()
    }
    fn get_joint_limits(&self) -> Vec<Option<Range<T>>> {
        let links_with_angle = self.id_list
            .iter()
            .filter(|id| self.tree.get(*id).data.has_joint_angle())
            .collect::<Vec<_>>();
        links_with_angle
            .iter()
            .map(|id| self.tree.get(id).data.joint.limits.clone())
            .collect()
    }
    /// skip fixed joint
    fn get_joint_names(&self) -> Vec<String> {
        let links_with_angle = self.id_list
            .iter()
            .filter(|id| self.tree.get(id).data.has_joint_angle())
            .collect::<Vec<_>>();
        links_with_angle
            .iter()
            .map(|id| self.tree.get(id).data.joint.name.to_string())
            .collect()
    }
}

/// Kinematic Tree using `IdTree<Link<T>>`
pub struct IdLinkTree<T: Real> {
    pub name: String,
    pub tree: IdTree<Link<T>>,
}

impl<T: Real> IdLinkTree<T> {
    /// Create LinkTree from root link
    ///
    /// # Arguments
    ///
    /// * `root_link` - root node of the links
    pub fn new(name: &str, tree: IdTree<Link<T>>) -> Self {
        Self {
            name: name.to_string(),
            tree,
        }
    }
    pub fn get_root_node_id(&self) -> NodeId {
        for node in self.tree.iter() {
            if node.parent.is_none() {
                return node.id.clone();
            }
        }
        assert!(false, "could not found root");
        NodeId(0)
    }
    /// Set the transform of the root link
    pub fn set_root_transform(&mut self, transform: Isometry3<T>) {
        self.tree.get_mut(&NodeId(0)).data.transform = transform;
    }
    /// iter for all link nodes
    pub fn iter(&self) -> Iter<IdLink<T>> {
        self.tree.iter()
    }
    /// iter for all link nodes as mut
    pub fn iter_mut(&mut self) -> IterMut<IdLink<T>> {
        self.tree.iter_mut()
    }
    /// iter for the links with the joint which is not fixed
    pub fn iter_joints<'a>(&'a self) -> Box<Iterator<Item = &IdLink<T>> + 'a> {
        Box::new(self.iter().filter(|node| node.data.has_joint_angle()))
    }
    /// iter for the links with the joint which is not fixed
    pub fn iter_joints_mut<'a>(&'a mut self) -> Box<Iterator<Item = &mut IdLink<T>> + 'a> {
        Box::new(self.iter_mut().filter(|node| node.data.has_joint_angle()))
    }
    /// Get the degree of freedom
    pub fn dof(&self) -> usize {
        self.iter_joints().count()
    }
}


impl<T> JointContainer<T> for IdLinkTree<T>
where
    T: Real,
{
    /// Get the angles of the joints
    ///
    /// `FixedJoint` is ignored. the length is the same with `dof()`
    fn get_joint_angles(&self) -> Vec<T> {
        self.iter()
            .filter_map(|node| node.data.get_joint_angle())
            .collect()
    }

    /// Set the angles of the joints
    ///
    /// `FixedJoints` are ignored. the input number must be equal with `dof()`
    fn set_joint_angles(&mut self, angles_vec: &[T]) -> Result<(), JointError> {
        if angles_vec.len() != self.dof() {
            return Err(JointError::SizeMisMatch);
        }
        for (node, angle) in self.iter_joints_mut().zip(angles_vec.iter()) {
            node.data.set_joint_angle(*angle)?;
        }
        Ok(())
    }

    fn get_joint_limits(&self) -> Vec<Option<Range<T>>> {
        self.iter_joints()
            .map(|node| node.data.joint.limits.clone())
            .collect()
    }
    fn get_joint_names(&self) -> Vec<String> {
        self.iter_joints()
            .map(|node| node.data.joint.name.clone())
            .collect()
    }
}

impl<T> LinkContainer<T> for IdLinkTree<T>
where
    T: Real,
{
    fn calc_link_transforms(&self) -> Vec<Isometry3<T>> {
        self.tree
            .iter_descendants(&NodeId(0))
            .map(|node| {
                let parent_transform = match node.parent {
                    Some(ref parent) => {
                        match *self.tree.get(parent).data.world_transform_cache.borrow() {
                            Some(trans) => trans,
                            None => Isometry3::identity(),
                        }
                    }
                    None => Isometry3::identity(),
                };
                let trans = parent_transform * node.data.calc_transform();
                *node.data.world_transform_cache.borrow_mut() = Some(trans);
                trans
            })
            .collect()
    }
    fn get_link_names(&self) -> Vec<String> {
        self.iter().map(|node| node.data.name.to_owned()).collect()
    }
}

impl<'a, T> CreateChain<'a, IdKinematicChain<'a, T>, T> for IdLinkTree<T>
where
    T: Real,
{
    fn chain_from_end_link_name(
        &'a mut self,
        end_link_name: &str,
    ) -> Option<IdKinematicChain<'a, T>> {
        let ids;
        {
            let end_node_opt = self.tree.iter().find(
                |node| node.data.name == end_link_name,
            );
            ids = match end_node_opt {
                Some(end_node) => {
                    let mut node_ids = self.tree
                        .iter_ancestors(&end_node.id)
                        .map(|node| node.id)
                        .collect::<Vec<_>>();
                    node_ids.reverse();
                    Some(node_ids)
                }
                None => None,
            };
        }
        match ids {
            Some(ids) => Some(IdKinematicChain::<'a, T>::new(
                end_link_name,
                &mut self.tree,
                &ids,
            )),
            None => None,
        }
    }
}

#[test]
fn it_works() {
    let l0 = LinkBuilder::new()
        .name("link0")
        .translation(na::Translation3::new(0.0, 0.1, 0.0))
        .joint(
            "j0",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l1 = LinkBuilder::new()
        .name("link1")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j1",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l2 = LinkBuilder::new()
        .name("link2")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j2",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l3 = LinkBuilder::new()
        .name("link3")
        .translation(na::Translation3::new(0.0, 0.1, 0.2))
        .joint(
            "j3",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l4 = LinkBuilder::new()
        .name("link4")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j4",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let l5 = LinkBuilder::new()
        .name("link5")
        .translation(na::Translation3::new(0.0, 0.1, 0.1))
        .joint(
            "j5",
            JointType::Rotational { axis: na::Vector3::y_axis() },
            None,
        )
        .finalize();
    let mut tree = IdTree::new();

    let ljn0 = tree.create_node(l0);
    let ljn1 = tree.create_node(l1);
    let ljn2 = tree.create_node(l2);
    let ljn3 = tree.create_node(l3);
    let ljn4 = tree.create_node(l4);
    let ljn5 = tree.create_node(l5);
    tree.set_parent_child(&ljn0, &ljn1);
    tree.set_parent_child(&ljn1, &ljn2);
    tree.set_parent_child(&ljn2, &ljn3);
    tree.set_parent_child(&ljn0, &ljn4);
    tree.set_parent_child(&ljn4, &ljn5);
    let robot = IdLinkTree::new("robo1", tree);
    assert_eq!(robot.dof(), 6);
}
