use std::cell::{Ref, RefMut, RefCell};
use std::slice::{Iter, IterMut};

#[derive(Copy, Clone, Debug)]
pub struct NodeId(usize);

#[derive(Debug)]
pub struct IdNode<T> {
    pub parent: Option<NodeId>,
    pub children: Vec<NodeId>,
    pub little_sister: Option<NodeId>,
    pub id: NodeId,
    pub data: T,
}

impl<T> IdNode<T> {
    pub fn new(data: T, id: NodeId) -> Self {
        Self {
            parent: None,
            children: Vec::new(),
            little_sister: None,
            id,
            data,
        }
    }
}

pub type IdNodeCell<T> = RefCell<IdNode<T>>;


pub struct NodeIter<'a, T: 'a> {
    iter: Iter<'a, IdNodeCell<T>>,
}

impl<'a, T: 'a> Iterator for NodeIter<'a, T> {
    type Item = Ref<'a, T>;

    fn next(&mut self) -> Option<Ref<'a, T>> {
        self.iter.next().map(|rc| {
            Ref::map(rc.borrow(), |node| &node.data)
        })
    }
}

pub struct NodeIterMut<'a, T: 'a> {
    iter: IterMut<'a, IdNodeCell<T>>,
}

impl<'a, T: 'a> Iterator for NodeIterMut<'a, T> {
    type Item = RefMut<'a, T>;

    fn next(&mut self) -> Option<RefMut<'a, T>> {
        self.iter.next().map(|rc| {
            RefMut::map(rc.borrow_mut(), |node| &mut node.data)
        })
    }
}

pub struct Ancestor<'a, T>
where
    T: 'a,
{
    tree: &'a IdTree<T>,
    id: Option<NodeId>,
}

impl<'a, T> Iterator for Ancestor<'a, T> {
    type Item = Ref<'a, IdNode<T>>;

    fn next(&mut self) -> Option<Ref<'a, IdNode<T>>> {
        if let Some(id) = self.id {
            let next = self.tree.get(id);
            match next.parent {
                None => self.id = None,
                Some(parent) => self.id = Some(parent),
            }
            Some(next)
        } else {
            None
        }
    }
}


#[derive(Debug)]
pub struct IdTree<T> {
    nodes: Vec<IdNodeCell<T>>,
}


impl<T> IdTree<T> {
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }
    pub fn create_node(&mut self, data: T) -> NodeId {
        let id = NodeId(self.nodes.len());
        self.nodes.push(RefCell::new(IdNode::new(data, id)));
        id
    }
    pub fn set_parent_child(&self, parent_id: NodeId, child_id: NodeId) {
        let parent = &self.nodes[parent_id.0];
        let child = &self.nodes[child_id.0];
        let parent_children = &mut parent.borrow_mut().children;
        if !parent_children.is_empty() {
            let sister_id = parent_children.last().unwrap();
            let sister = &self.nodes[sister_id.0];
            sister.borrow_mut().little_sister = Some(child_id);
        }
        parent_children.push(child_id);
        child.borrow_mut().parent = Some(parent_id);
    }
    pub fn get(&self, id: NodeId) -> Ref<IdNode<T>> {
        self.nodes[id.0].borrow()
    }
    pub fn get_mut(&self, id: NodeId) -> RefMut<IdNode<T>> {
        self.nodes[id.0].borrow_mut()
    }
    /// iter for all link nodes
    pub fn iter(&self) -> Iter<IdNodeCell<T>> {
        self.nodes.iter()
    }
    /// iter for all link nodes as mut
    pub fn iter_mut(&mut self) -> IterMut<IdNodeCell<T>> {
        self.nodes.iter_mut()
    }
    /// iter for all link nodes
    pub fn iter_data<'a>(&'a self) -> NodeIter<'a, T> {
        NodeIter { iter: self.iter() }
    }
    /// iter for all link nodes as mut
    pub fn iter_data_mut<'a>(&'a mut self) -> NodeIterMut<'a, T> {
        NodeIterMut { iter: self.iter_mut() }
    }
    /// iter from the end to root, it contains nodes[id] itsself
    pub fn iter_ancestors(&self, id: NodeId) -> Ancestor<T> {
        Ancestor {
            tree: self,
            id: Some(id),
        }
    }
}

#[test]
fn test_idtree() {
    let mut tree = IdTree::<String>::new();
    let n0 = tree.create_node("hoge0".to_owned());
    let n1 = tree.create_node("hoge1".to_owned());
    let n2 = tree.create_node("hoge2".to_owned());
    let n3 = tree.create_node("hoge3".to_owned());
    tree.set_parent_child(n0, n1);
    tree.set_parent_child(n1, n2);
    tree.set_parent_child(n1, n3);

    assert_eq!(tree.get(n0).data, "hoge0");
    assert_eq!(tree.get(n1).children.len(), 2);
    assert!(tree.get(n1).parent.is_some());
    assert!(tree.get(n0).parent.is_none());
    assert_eq!(tree.get(n2).children.len(), 0);
    tree.get_mut(n1).data = "aaa".to_owned();
    assert_eq!(tree.get(n1).data, "aaa");
    let data = tree.iter()
        .map(|ref_node| ref_node.borrow().data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge0", "aaa", "hoge2", "hoge3"]);
    let data = tree.iter_data()
        .map(|data| data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge0", "aaa", "hoge2", "hoge3"]);

    for mut ref_data in tree.iter_data_mut() {
        ref_data.push_str("_");
    }
    let data = tree.iter_data()
        .map(|data| data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge0_", "aaa_", "hoge2_", "hoge3_"]);

    assert_eq!(tree.iter_ancestors(n0).count(), 1);
    assert_eq!(tree.iter_ancestors(n2).count(), 3);
    let data = tree.iter_ancestors(n2)
        .map(|node| node.data.clone())
        .collect::<Vec<_>>();
    assert_eq!(data, vec!["hoge2_", "aaa_", "hoge0_"]);
}