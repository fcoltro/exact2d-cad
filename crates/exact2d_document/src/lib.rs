//! Phase 4.1 — Document model for Exact2D CAD.
//!
//! Layers, blocks, entities (geometry + properties), line types, views, settings.
//! Entities retain their exact algebraic geometry; edits apply exact transforms.

pub mod properties;
pub mod layer;
pub mod dimension;
pub mod entity;
pub mod document;

pub use properties::{Color, LineWeight, LineTypeRef, LineTypeDef, XData};
pub use layer::{Layer, LayerTable};
pub use dimension::{Dimension, DimKind, DimStyle, DimStyleTable, LinearOrient};
pub use entity::{Entity, EntityId, EntityKind};
pub use document::{Document, Block, NamedView, Settings, Units};
