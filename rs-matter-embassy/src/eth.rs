//! Eternet: Type aliases for an Embassy Matter stack running over an Ethernet network (or any other network not managed by Matter).

use rs_matter_stack::{Eth, MatterStack};

/// A type alias for an Embassy Matter stack running over an Ethernet network (or any other network not managed by Matter).
pub type EmbassyEthMatterStack<'a, E = ()> = MatterStack<'a, EmbassyEth<E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// an Ethernet network (or any other network not managed by Matter).
pub type EmbassyEth<E> = Eth<E>;
