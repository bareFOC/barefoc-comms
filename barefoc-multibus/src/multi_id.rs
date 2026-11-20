//! MultiBus Multi-ID routing for multi-joint actuator nodes.
//!
//! Provides [`MultiIdRouter`] and [`RouteTarget`] to route incoming packet IDs
//! to the composite board, an individual joint axis, broadcast, or ignore.

/// Routing target for incoming bus packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteTarget {
    /// Addressed to the composite board controller.
    Board,
    /// Addressed to a specific joint axis by local index.
    Joint(usize),
    /// Broadcast to all nodes on the bus.
    Broadcast,
    /// Not addressed to this board or its joints.
    Ignore,
}

/// Standard Dynamixel / STS broadcast address.
pub const DEFAULT_BROADCAST_ID: u8 = 0xFE;

/// Multi-ID router for a board supervising up to `N` joint axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MultiIdRouter<const N: usize = 4> {
    board_id: u8,
    joint_ids: [u8; N],
    broadcast_id: u8,
}

impl<const N: usize> Default for MultiIdRouter<N> {
    fn default() -> Self {
        Self {
            board_id: 1,
            joint_ids: [0; N],
            broadcast_id: DEFAULT_BROADCAST_ID,
        }
    }
}

impl<const N: usize> MultiIdRouter<N> {
    /// Create a new router with given board ID and joint IDs.
    pub const fn new(board_id: u8, joint_ids: [u8; N]) -> Self {
        Self {
            board_id,
            joint_ids,
            broadcast_id: DEFAULT_BROADCAST_ID,
        }
    }

    /// Override the broadcast identifier.
    pub const fn with_broadcast_id(mut self, broadcast_id: u8) -> Self {
        self.broadcast_id = broadcast_id;
        self
    }

    /// Resolve an incoming packet ID into a [`RouteTarget`].
    pub fn resolve(&self, packet_id: u8) -> RouteTarget {
        if packet_id == self.broadcast_id {
            return RouteTarget::Broadcast;
        }
        if packet_id == self.board_id {
            return RouteTarget::Board;
        }
        for (idx, &id) in self.joint_ids.iter().enumerate() {
            if packet_id == id {
                return RouteTarget::Joint(idx);
            }
        }
        RouteTarget::Ignore
    }

    /// Get current board ID.
    pub const fn board_id(&self) -> u8 {
        self.board_id
    }

    /// Set board ID.
    pub fn set_board_id(&mut self, id: u8) {
        self.board_id = id;
    }

    /// Get joint IDs.
    pub const fn joint_ids(&self) -> &[u8; N] {
        &self.joint_ids
    }

    /// Update a joint ID at given index.
    pub fn set_joint_id(&mut self, idx: usize, id: u8) -> bool {
        if idx < N {
            self.joint_ids[idx] = id;
            true
        } else {
            false
        }
    }

    /// Check if packet ID is addressed to this node (Board, Joint, or Broadcast).
    pub fn is_addressed(&self, packet_id: u8) -> bool {
        self.resolve(packet_id) != RouteTarget::Ignore
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multi_id_router_resolve() {
        let router = MultiIdRouter::new(10, [1, 2, 3, 4]);

        assert_eq!(router.resolve(0xFE), RouteTarget::Broadcast);
        assert_eq!(router.resolve(10), RouteTarget::Board);
        assert_eq!(router.resolve(1), RouteTarget::Joint(0));
        assert_eq!(router.resolve(2), RouteTarget::Joint(1));
        assert_eq!(router.resolve(3), RouteTarget::Joint(2));
        assert_eq!(router.resolve(4), RouteTarget::Joint(3));
        assert_eq!(router.resolve(99), RouteTarget::Ignore);
    }

    #[test]
    fn test_multi_id_router_mutate() {
        let mut router = MultiIdRouter::new(0x20, [0x21, 0x22]);
        assert_eq!(router.resolve(0x21), RouteTarget::Joint(0));

        assert!(router.set_joint_id(0, 0x31));
        assert_eq!(router.resolve(0x21), RouteTarget::Ignore);
        assert_eq!(router.resolve(0x31), RouteTarget::Joint(0));

        router.set_board_id(0x40);
        assert_eq!(router.resolve(0x40), RouteTarget::Board);
    }
}
