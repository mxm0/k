//use std::cell::{Ref, RefMut, RefCell};
use std::slice::{Iter, IterMut};

#[derive(Copy, Clone, Debug)]
pub struct NodeId(pub usize);

#[derive(Debug)]
pub struct IdNode<T> {
    pub parent: Option<NodeId>,
    pub children: Vec<NodeId>,
    pub id: NodeId,
    pub data: T,
}

impl<T> IdNode<T> {
    pub fn new(data: T, id: NodeId) -> Self {
        Self {
            parent: None,
            children: Vec::new(),
            id,
            data,
        }
    }
}

pub struct Ancestors<'a, T>
where
    T: 'a,
{
    tree: &'a IdTree<T>,
    parent_id: Option<NodeId>,
}

impl<'a, T> Iterator for Ancestors<'a, T> {
    type Item = &'a IdNode<T>;

    fn next(&mut self) -> Option<&'a IdNode<T>> {
        if let Some(ref id) = self.parent_id.clone() {
            let next = self.tree.get(id);
            match next.parent {
                None => self.parent_id = None,
                Some(parent) => self.parent_id = Some(parent),
            }
            Some(next)
        } else {
            None
        }
    }
}

pub struct Descendants<'a, T>
where
    T: 'a,
{
    tree: &'a IdTree<T>,
    stack: Vec<NodeId>,
}

impl<'a, T> Iterator for Descendants<'a, T> {
    type Item = &'a IdNode<T>;

    fn next(&mut self) -> Option<Self::Item> {
        let ret_id = match self.stack.pop() {
            Some(id) => id,
            None => {
                return None;
            }
        };
        let node = self.tree.get(&ret_id);
        self.stack.extend(node.children.clone());
        Some(node)
    }
}

#[derive(Debug)]
/// Tree structure which has an arena of `IdNode<T>`
pub struct IdTree<T> {
    nodes: Vec<IdNode<T>>,
}

impl<T> IdTree<T> {
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }
    pub fn create_node(&mut self, data: T) -> NodeId {
        let id = NodeId(self.nodes.len());
        self.nodes.push(IdNode::new(data, id));
        id
    }
    pub fn set_parent_child(&mut self, parent_id: &NodeId, child_id: &NodeId) {
        self.nodes[child_id.0].parent = Some(parent_id.clone());
        self.nodes[parent_id.0].children.push(child_id.clone());
    }
    pub fn get(&self, id: &NodeId) -> &IdNode<T> {
        &self.nodes[id.0]
    }
    pub fn get_mut(&mut self, id: &NodeId) -> &mut IdNode<T> {
        &mut self.nodes[id.0]
    }
    /// iter for all link nodes
    pub fn iter(&self) -> Iter<IdNode<T>> {
        self.nodes.iter()
    }
    /// iter for all link nodes as mut
    pub fn iter_mut(&mut self) -> IterMut<IdNode<T>> {
        self.nodes.iter_mut()
    }
    /// iter from the end to root, it contains nodes[id] itsself
    pub fn iter_ancestors(&self, id: &NodeId) -> Ancestors<T> {
        Ancestors {
            tree: self,
            parent_id: Some(id.clone()),
        }
    }
    /// iter from the root to end, it contains nodes[id] itsself
    pub fn iter_descendants(&self, id: &NodeId) -> Descendants<T> {
        Descendants {
            tree: self,
            stack: vec![id.clone()],
        }
    }
    pub fn get_root_node_id(&self) -> NodeId {
        for node in self.iter() {
            if node.parent.is_none() {
                return node.id.clone();
            }
        }
        panic!("could not found root");
    }
}

#[test]
fn test_idtree() {
    let mut tree = IdTree::<String>::new();
    let n0 = tree.create_node("hoge0".to_owned());
    let n1 = tree.create_node("hoge1".to_owned());
    let n2 = tree.create_node("hoge2".to_owned());
    let n3 = tree.create_node("hoge3".to_owned());
    tree.set_parent_child(&n0, &n1);
    tree.set_parent_child(&n1, &n2);
    tree.set_parent_child(&n1, &n3);

    assert_eq!(tree.get(&n0).data, "hoge0");
    assert_eq!(tree.get(&n1).children.len(), 2);
    assert!(tree.get(&n1).parent.is_some());
    assert!(tree.get(&n0).parent.is_none());
    assert_eq!(tree.get(&n2).children.len(), 0);
    tree.get_mut(&n1).data = "aaa".to_owned();
    assert_eq!(tree.get(&n1).data, "aaa");

    let data = tree.iter()
        .map(|node| node.data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge0", "aaa", "hoge2", "hoge3"]);
    for ref_data in tree.iter_mut() {
        ref_data.data.push_str("_");
    }
    let data = tree.iter()
        .map(|node| node.data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge0_", "aaa_", "hoge2_", "hoge3_"]);

    assert_eq!(tree.iter_ancestors(&n0).count(), 1);
    assert_eq!(tree.iter_ancestors(&n2).count(), 3);
    let data = tree.iter_ancestors(&n2)
        .map(|node| node.data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge2_", "aaa_", "hoge0_"]);

    assert_eq!(tree.iter_descendants(&n0).count(), 4);
    let data = tree.iter_descendants(&n1)
        .map(|node| node.data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["aaa_", "hoge3_", "hoge2_"]);

}