use anyhow::Result;
use clap::Parser;
use eframe::{NativeOptions, egui::TextureId};
use egui::{Color32, Key, Rect, Sense, Stroke, Vec2, pos2, Pos2};
use eframe::epaint::TextureHandle;
use petgraph::graph::{DiGraph, NodeIndex};
use petgraph::visit::EdgeRef;
use rayon::prelude::*;
use serde::Deserialize;
use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use walkdir::WalkDir;
use zip::ZipArchive;
use image::DynamicImage;

fn normalize_vec(v: Vec2) -> Vec2 {
    let length = v.length();
    if length > 0.0 {
        Vec2::new(v.x / length, v.y / length)
    } else {
        Vec2::new(1.0, 0.0)
    }
}

fn rotate_vec(v: Vec2, angle: f32) -> Vec2 {
    let cos = angle.cos();
    let sin = angle.sin();
    Vec2::new(
        v.x * cos - v.y * sin,
        v.x * sin + v.y * cos,
    )
}

#[derive(Parser, Debug)]
#[command(
    name = "modsdepgraph",
    about = "Generate a dependency graph for Minecraft mods"
)]
struct Args {
    #[arg(
        help = "Path to the mods directory",
        default_value = "C:\\Users\\MidnightTale\\AppData\\Roaming\\PrismLauncher\\instances\\Luminance\\minecraft\\mods"
    )]
    mods_path: PathBuf,
}

// NeoForge/Forge mod structure
#[derive(Debug, Deserialize)]
struct ModToml {
    #[serde(default)]
    mods: Vec<Mod>,
    #[serde(default)]
    dependencies: HashMap<String, Vec<Dependency>>,
}

#[derive(Debug, Deserialize, Clone)]
struct Mod {
    #[serde(rename = "modId")]
    mod_id: String,
    version: String,
    #[serde(rename = "displayName", default)]
    display_name: String,
    #[serde(rename = "logoFile", default)]
    logo_file: Option<String>,
}

#[derive(Debug, Deserialize)]
struct Dependency {
    #[serde(rename = "modId")]
    mod_id: String,
    #[serde(rename = "type")]
    dependency_type: String,
    #[serde(rename = "versionRange", default)]
    #[allow(dead_code)]
    version_range: String,
    #[serde(rename = "side", default)]
    side: Option<String>,
}

// Fabric mod structure
#[derive(Debug, Deserialize)]
struct FabricMod {
    id: String,
    version: String,
    name: Option<String>,
    #[serde(default)]
    depends: HashMap<String, String>,
    #[serde(default)]
    recommends: HashMap<String, String>,
    #[serde(default)]
    suggests: HashMap<String, String>,
    #[serde(default)]
    conflicts: HashMap<String, String>,
    #[serde(default)]
    breaks: HashMap<String, String>,
    #[serde(default)]
    icon: Option<String>,
    #[serde(default)]
    environment: Option<String>,
}

#[derive(Debug)]
enum ModFormat {
    NeoForge(ModToml),
    Fabric(FabricMod),
}

#[derive(Debug, Clone)]
struct ModIcon {
    mod_id: String,
    image_data: Vec<u8>,
}

#[derive(Debug)]
struct Node {
    id: NodeIndex,
    label: String,
    pos: Pos2,
    radius: f32,
    color: Color32,
    texture: Option<TextureId>,
}

#[derive(Debug)]
struct Edge {
    from: NodeIndex,
    to: NodeIndex,
    label: String,
    color: Color32,
    side: Option<String>,
}

struct AppData {
    mods: Vec<(String, ModFormat)>,
    graph: Option<DiGraph<String, String>>,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    status: String,
    rx: Receiver<AppMessage>,
    loaded: bool,
    mod_info: Option<(String, String)>,
    selected_node: Option<NodeIndex>,
    hovered_node: Option<usize>,
    search_text: String,
    matched_mods: Vec<String>,
    
    // View state
    zoom: f32,
    pan_offset: Vec2,
    dragging: bool,
    drag_start: Pos2,
    drag_node: Option<usize>,
    
    // Icons
    icons: HashMap<String, TextureHandle>,
    icon_data: Vec<ModIcon>,
}

enum AppMessage {
    ModsLoaded(Vec<(String, ModFormat)>, Vec<ModIcon>),
    Error(String),
    Progress(String),
}

struct ModDepGraphApp {
    data: AppData,
}

impl ModDepGraphApp {
    fn new(cc: &eframe::CreationContext<'_>, mods_path: PathBuf) -> Self {
        // Set up channel for background thread communication
        let (tx, rx) = channel();
        
        // Spawn background thread to load mods
        thread::spawn(move || {
            match scan_mods(&mods_path, tx.clone()) {
                Ok((mods, icons)) => {
                    tx.send(AppMessage::ModsLoaded(mods, icons)).unwrap();
                }
                Err(e) => {
                    tx.send(AppMessage::Error(format!("Error scanning mods: {}", e))).unwrap();
                }
            }
        });

        Self {
            data: AppData {
                mods: Vec::new(),
                graph: None,
                nodes: Vec::new(),
                edges: Vec::new(),
                status: "Loading mods...".to_string(),
                rx,
                loaded: false,
                mod_info: None,
                selected_node: None,
                hovered_node: None,
                search_text: String::new(),
                matched_mods: Vec::new(),
                zoom: 1.0,
                pan_offset: Vec2::ZERO,
                dragging: false,
                drag_start: Pos2::ZERO,
                drag_node: None,
                icons: HashMap::new(),
                icon_data: Vec::new(),
            },
        }
    }
    
    fn update_matched_mods(&mut self) {
        self.data.matched_mods.clear();
        
        if self.data.search_text.is_empty() {
            return;
        }
        
        let search_lower = self.data.search_text.to_lowercase();
        if let Some(graph) = &self.data.graph {
            for idx in graph.node_indices() {
                let node_label = &graph[idx];
                if node_label.to_lowercase().contains(&search_lower) {
                    self.data.matched_mods.push(node_label.clone());
                }
            }
        }
    }
    
    fn initialize_layout(&mut self) {
        if let Some(graph) = &self.data.graph {
            // Generate nodes
            self.data.nodes.clear();
            
            // Initial layout - use a spiral to avoid exact overlaps from the start
            let node_count = graph.node_count() as f32;
            let max_radius = 400.0;  // Larger initial radius to spread nodes more
            
            for (i, idx) in graph.node_indices().enumerate() {
                let node_label = &graph[idx];
                
                // Use a spiral layout for initial positions
                let angle = 2.0 * std::f32::consts::PI * (i as f32) / 20.0;  // More spacing between angles
                let distance_factor = (i as f32) / node_count;
                let distance = max_radius * (0.2 + 0.8 * distance_factor);  // Start from 20% radius
                
                let x = distance * angle.cos();
                let y = distance * angle.sin();
                
                // Set colors based on mod type
                let color = self.get_node_color_for_mod_id(&node_label);
                
                // Look for mod icon
                let texture = None; // We'll load textures in the update method
                
                self.data.nodes.push(Node {
                    id: idx,
                    label: node_label.clone(),
                    pos: pos2(x, y),
                    radius: 25.0, // Larger radius for icons
                    color,
                    texture,
                });
            }
            
            // Generate edges
            self.data.edges.clear();
            for edge in graph.edge_references() {
                // Determine color based on dependency type
                let dep_type = edge.weight();
                let color = self.get_edge_color(dep_type);
                
                // Extract side information
                let side = if dep_type.contains("CLIENT") {
                    Some("CLIENT".to_string())
                } else if dep_type.contains("SERVER") {
                    Some("SERVER".to_string())
                } else {
                    None
                };
                
                self.data.edges.push(Edge {
                    from: edge.source(),
                    to: edge.target(),
                    label: edge.weight().clone(),
                    color,
                    side,
                });
            }

            // Apply force-directed layout to reduce overlap
            self.apply_force_directed_layout(100); // More iterations for better spacing

            // Center the graph in the view
            let screen_center = Vec2::new(1280.0 / 2.0, 800.0 / 2.0);
            self.data.pan_offset = screen_center;
        }
    }
    
    fn apply_force_directed_layout(&mut self, iterations: usize) {
        let repulsion = 10000.0; // Stronger repulsive force between nodes
        let attraction = 0.005; // Weaker attractive force for edges
        let damping = 0.9; // Damping factor to prevent oscillation
        
        let mut velocities: Vec<Vec2> = vec![Vec2::ZERO; self.data.nodes.len()];
        
        for _ in 0..iterations {
            // Calculate forces
            let mut forces = vec![Vec2::ZERO; self.data.nodes.len()];
            
            // Repulsive forces (nodes repel each other)
            for i in 0..self.data.nodes.len() {
                for j in 0..self.data.nodes.len() {
                    if i != j {
                        let delta_x = self.data.nodes[i].pos.x - self.data.nodes[j].pos.x;
                        let delta_y = self.data.nodes[i].pos.y - self.data.nodes[j].pos.y;
                        
                        let distance_sq = delta_x * delta_x + delta_y * delta_y;
                        let distance = distance_sq.sqrt().max(1.0); // Avoid division by zero
                        
                        // Stronger repulsion for closer nodes
                        let force = if distance < 100.0 {
                            repulsion * 2.0 / distance_sq
                        } else {
                            repulsion / distance_sq
                        };
                        
                        let force_x = force * delta_x / distance;
                        let force_y = force * delta_y / distance;
                        
                        forces[i].x += force_x;
                        forces[i].y += force_y;
                    }
                }
            }
            
            // Attractive forces (edges pull connected nodes together)
            for edge in &self.data.edges {
                if let (Some(from_idx), Some(to_idx)) = (
                    self.data.nodes.iter().position(|n| n.id == edge.from),
                    self.data.nodes.iter().position(|n| n.id == edge.to),
                ) {
                    let delta_x = self.data.nodes[from_idx].pos.x - self.data.nodes[to_idx].pos.x;
                    let delta_y = self.data.nodes[from_idx].pos.y - self.data.nodes[to_idx].pos.y;
                    
                    let distance = (delta_x * delta_x + delta_y * delta_y).sqrt().max(1.0);
                    
                    let force = attraction * distance;
                    let force_x = force * delta_x / distance;
                    let force_y = force * delta_y / distance;
                    
                    forces[from_idx].x -= force_x;
                    forces[from_idx].y -= force_y;
                    
                    forces[to_idx].x += force_x;
                    forces[to_idx].y += force_y;
                }
            }
            
            // Apply forces and update positions
            for i in 0..self.data.nodes.len() {
                velocities[i].x = (velocities[i].x + forces[i].x) * damping;
                velocities[i].y = (velocities[i].y + forces[i].y) * damping;
                
                self.data.nodes[i].pos.x += velocities[i].x;
                self.data.nodes[i].pos.y += velocities[i].y;
            }
        }
        
        // Center the graph
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        
        for node in &self.data.nodes {
            min_x = min_x.min(node.pos.x);
            min_y = min_y.min(node.pos.y);
            max_x = max_x.max(node.pos.x);
            max_y = max_y.max(node.pos.y);
        }
        
        let center_x = (min_x + max_x) / 2.0;
        let center_y = (min_y + max_y) / 2.0;
        
        for node in &mut self.data.nodes {
            node.pos.x -= center_x;
            node.pos.y -= center_y;
        }
    }
    
    fn screen_to_world(&self, screen_pos: Pos2) -> Pos2 {
        let offset = self.data.pan_offset;
        let zoom = self.data.zoom;
        
        Pos2::new(
            (screen_pos.x - offset.x) / zoom,
            (screen_pos.y - offset.y) / zoom,
        )
    }
    
    fn world_to_screen(&self, world_pos: Pos2) -> Pos2 {
        let offset = self.data.pan_offset;
        let zoom = self.data.zoom;
        
        Pos2::new(
            world_pos.x * zoom + offset.x,
            world_pos.y * zoom + offset.y,
        )
    }
    
    fn find_node_at_pos(&self, pos: Pos2) -> Option<usize> {
        for (i, node) in self.data.nodes.iter().enumerate() {
            let screen_pos = self.world_to_screen(node.pos);
            let radius = node.radius * self.data.zoom;
            
            // Square hitbox instead of circular
            let rect = Rect::from_center_size(
                screen_pos,
                Vec2::new(radius * 2.0, radius * 2.0)
            );
            
            if rect.contains(pos) {
                return Some(i);
            }
        }
        None
    }
    
    fn select_node(&mut self, node_idx: usize) {
        let node = &self.data.nodes[node_idx];
        self.data.selected_node = Some(node.id);
        
        if let Some(graph) = &self.data.graph {
            let mod_name = &graph[node.id];
            
            // Find incoming and outgoing dependencies
            let mut dependencies = Vec::new();
            let mut dependents = Vec::new();
            
            for edge in graph.edges_directed(node.id, petgraph::Direction::Outgoing) {
                let side_info = if let Some(edge_idx) = self.data.edges.iter().position(|e| 
                    e.from == edge.source() && e.to == edge.target()) {
                    if let Some(side) = &self.data.edges[edge_idx].side {
                        format!(" [{}]", side)
                    } else {
                        String::new()
                    }
                } else {
                    String::new()
                };
                
                dependencies.push(format!("{} → {} ({}{})", 
                    graph[edge.source()], 
                    graph[edge.target()], 
                    edge.weight(),
                    side_info));
            }
            
            for edge in graph.edges_directed(node.id, petgraph::Direction::Incoming) {
                let side_info = if let Some(edge_idx) = self.data.edges.iter().position(|e| 
                    e.from == edge.source() && e.to == edge.target()) {
                    if let Some(side) = &self.data.edges[edge_idx].side {
                        format!(" [{}]", side)
                    } else {
                        String::new()
                    }
                } else {
                    String::new()
                };
                
                dependents.push(format!("{} → {} ({}{})", 
                    graph[edge.source()], 
                    graph[edge.target()], 
                    edge.weight(),
                    side_info));
            }
            
            let mut info = String::new();
            
            if !dependencies.is_empty() {
                info.push_str("Dependencies:\n");
                for dep in dependencies {
                    info.push_str(&format!("• {}\n", dep));
                }
                info.push('\n');
            }
            
            if !dependents.is_empty() {
                info.push_str("Required by:\n");
                for dep in dependents {
                    info.push_str(&format!("• {}\n", dep));
                }
            }
            
            self.data.mod_info = Some((mod_name.clone(), info));
        }
    }

    fn get_edge_color(&self, dependency_type: &str) -> Color32 {
        match dependency_type.to_lowercase().as_str() {
            "required" => Color32::from_rgb(220, 50, 50),     // Red for required
            "optional" => Color32::from_rgb(100, 200, 100),   // Green for optional
            "recommends" => Color32::from_rgb(100, 180, 100), // Light green for recommends
            "suggests" => Color32::from_rgb(100, 220, 100),   // Bright green for suggests
            "conflicts" => Color32::from_rgb(220, 50, 50),    // Red for conflicts
            "breaks" => Color32::from_rgb(220, 100, 50),      // Orange for breaks
            _ => Color32::from_rgb(150, 150, 150),            // Default gray
        }
    }

    // New function to determine node color based on mod ID
    fn get_node_color_for_mod_id(&self, node_label: &str) -> Color32 {
        // Extract mod ID from the label format "Name (version)"
        if node_label.contains("Create") {
            Color32::from_rgb(220, 180, 0)  // Golden for Create
        } else if node_label.contains("Sodium") {
            Color32::from_rgb(120, 200, 255) // Light blue for Sodium
        } else if node_label.contains("Flywheel") || node_label.contains("Ponder") {
            Color32::from_rgb(200, 160, 0)  // Similar to Create for related mods
        } else if node_label.contains("Fabric") {
            Color32::from_rgb(70, 160, 200)  // Blue for Fabric mods
        } else if node_label.contains("Forge") || node_label.contains("forge") {
            Color32::from_rgb(200, 100, 70)  // Red for Forge mods
        } else if node_label.contains("Lithium") {
            Color32::from_rgb(150, 250, 150) // Green for optimization mods
        } else {
            // Default to standard blue
            Color32::from_rgb(100, 150, 250)
        }
    }

    fn load_icons(&mut self, ctx: &egui::Context) {
        let icons_to_load: Vec<ModIcon> = self.data.icon_data.iter()
            .filter(|icon| !self.data.icons.contains_key(&icon.mod_id))
            .cloned()
            .collect();
        
        if icons_to_load.is_empty() {
            return;
        }
        
        println!("Loading {} icons in parallel", icons_to_load.len());
        
        // Process icons in parallel using rayon
        let processed_icons: Vec<(String, Option<(Vec<u8>, [usize; 2])>)> = icons_to_load.par_iter()
            .map(|icon| {
                let mod_id = icon.mod_id.clone();
                let result = load_image_from_memory(&icon.image_data)
                    .map(|image| {
                        // Resize the image to a reasonable size
                        let max_size = 256;
                        let image = if image.width() > max_size || image.height() > max_size {
                            let width = image.width();
                            let height = image.height();
                            let aspect_ratio = width as f32 / height as f32;
                            
                            let (new_width, new_height) = if width > height {
                                (max_size, (max_size as f32 / aspect_ratio) as u32)
                            } else {
                                ((max_size as f32 * aspect_ratio) as u32, max_size)
                            };
                            
                            image.resize_exact(new_width, new_height, image::imageops::FilterType::Lanczos3)
                        } else {
                            image
                        };
                        
                        let size = [image.width() as usize, image.height() as usize];
                        let image_buffer = image.to_rgba8();
                        let pixels = image_buffer.as_flat_samples();
                        
                        (pixels.as_slice().to_vec(), size)
                    })
                    .ok();
                
                (mod_id, result)
            })
            .collect();
        
        // Now create the textures in the main thread
        for (mod_id, image_data) in processed_icons {
            if let Some((pixels, size)) = image_data {
                let texture = ctx.load_texture(
                    &mod_id,
                    egui::ColorImage::from_rgba_unmultiplied(
                        size,
                        &pixels,
                    ),
                    egui::TextureOptions::default(),
                );
                
                self.data.icons.insert(mod_id.clone(), texture);
                println!("Loaded icon for mod_id: {}", mod_id);
            }
        }
        
        // Update nodes with textures
        let mut icon_assignments = 0;
        for node in &mut self.data.nodes {
            // Extract mod ID from label
            let label = &node.label;
            let mod_id = if let Some(start_idx) = label.find('(') {
                label[0..start_idx].trim().to_lowercase()
            } else {
                label.to_lowercase()
            };
            
            // Find matching icon
            for (mod_id_key, texture) in &self.data.icons {
                let key_lower = mod_id_key.to_lowercase();
                if mod_id.contains(&key_lower) || key_lower.contains(&mod_id) {
                    node.texture = Some(texture.id());
                    icon_assignments += 1;
                    println!("Assigned icon to node: {} (using icon from: {})", label, mod_id_key);
                    break;
                }
            }
        }
        println!("Assigned icons to {} nodes out of {}", icon_assignments, self.data.nodes.len());
    }

    // Add a new function to fit all nodes into the view
    fn fit_graph_to_view(&mut self, view_width: f32, view_height: f32) {
        if self.data.nodes.is_empty() {
            return;
        }
        
        // Find bounds of all nodes
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        
        for node in &self.data.nodes {
            min_x = min_x.min(node.pos.x);
            min_y = min_y.min(node.pos.y);
            max_x = max_x.max(node.pos.x);
            max_y = max_y.max(node.pos.y);
        }
        
        // Add padding
        let padding = 50.0;
        min_x -= padding;
        min_y -= padding;
        max_x += padding;
        max_y += padding;
        
        // Calculate zoom to fit
        let graph_width = max_x - min_x;
        let graph_height = max_y - min_y;
        
        let zoom_x = view_width / graph_width;
        let zoom_y = view_height / graph_height;
        
        // Use the smaller zoom to ensure everything fits
        self.data.zoom = zoom_x.min(zoom_y).min(2.0);
        
        // Center the graph
        let graph_center_x = (min_x + max_x) / 2.0;
        let graph_center_y = (min_y + max_y) / 2.0;
        
        // Calculate the center of the view
        let view_center_x = view_width / 2.0;
        let view_center_y = view_height / 2.0;
        
        // Set pan offset to center the graph
        self.data.pan_offset = Vec2::new(
            view_center_x - graph_center_x * self.data.zoom,
            view_center_y - graph_center_y * self.data.zoom
        );
    }
}

impl eframe::App for ModDepGraphApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Check for messages from the background thread
        while let Ok(message) = self.data.rx.try_recv() {
            match message {
                AppMessage::ModsLoaded(mods, icons) => {
                    self.data.mods = mods;
                    self.data.icon_data = icons;
                    self.data.status = "Generating graph...".to_string();
                    
                    match generate_dependency_graph(&self.data.mods) {
                        Ok(graph) => {
                            if graph.node_count() > 0 {
                                self.data.graph = Some(graph);
                                self.initialize_layout();
                                self.data.status = format!("Loaded {} mods", self.data.mods.len());
                                self.data.loaded = true;
                            } else {
                                self.data.status = "No mod dependencies found".to_string();
                            }
                        }
                        Err(e) => {
                            self.data.status = format!("Error generating graph: {}", e);
                        }
                    }
                }
                AppMessage::Error(error) => {
                    self.data.status = error;
                }
                AppMessage::Progress(msg) => {
                    self.data.status = msg;
                    ctx.request_repaint();
                }
            }
        }

        // Load icons after graph is created
        if self.data.loaded && !self.data.icon_data.is_empty() {
            self.load_icons(ctx);
        }

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Minecraft Mod Dependency Graph");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(&self.data.status);
                });
            });
            
            ui.separator();
            
            ui.horizontal(|ui| {
                ui.label("Search:");
                if ui.text_edit_singleline(&mut self.data.search_text).changed() {
                    self.update_matched_mods();
                }
                
                if !self.data.search_text.is_empty() && !self.data.matched_mods.is_empty() {
                    ui.label(format!("Found {} matches", self.data.matched_mods.len()));
                }
            });
        });
        
        if !self.data.matched_mods.is_empty() {
            egui::Window::new("Search Results")
                .auto_sized()
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        // Create a copy of matched_mods to avoid borrow conflicts
                        let matched_mods = self.data.matched_mods.clone();
                        for mod_name in matched_mods {
                            if ui.selectable_label(false, &mod_name).clicked() {
                                // Find and select the node
                                for (i, node) in self.data.nodes.iter().enumerate() {
                                    if &node.label == &mod_name {
                                        self.select_node(i);
                                        break;
                                    }
                                }
                            }
                        }
                    });
                });
        }
        
        if let Some(info) = &self.data.mod_info {
            egui::Window::new("Mod Information")
                .auto_sized()
                .show(ctx, |ui| {
                    ui.heading(&info.0);
                    ui.separator();
                    ui.label(&info.1);
                });
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            if !self.data.loaded {
                ui.centered_and_justified(|ui| {
                    ui.spinner();
                    ui.label(&self.data.status);
                });
                return;
            }
            
            // Draw the graph
            let (response, painter) =
                ui.allocate_painter(ui.available_size_before_wrap(), Sense::click_and_drag());
            
            // Draw background
            let rect = response.rect;
            painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 20, 30));
            
            // Debug info - always show node and edge count
            painter.text(
                pos2(10.0, 20.0),
                egui::Align2::LEFT_TOP,
                format!("Nodes: {}, Edges: {}, Pan: ({:.1}, {:.1}), Zoom: {:.1}x", 
                    self.data.nodes.len(), 
                    self.data.edges.len(),
                    self.data.pan_offset.x,
                    self.data.pan_offset.y,
                    self.data.zoom
                ),
                egui::FontId::proportional(14.0),
                Color32::WHITE,
            );
            
            // If there are no nodes, show a message
            if self.data.nodes.is_empty() {
                painter.text(
                    pos2(rect.width() / 2.0, rect.height() / 2.0),
                    egui::Align2::CENTER_CENTER,
                    "No mod dependencies found. Try selecting a different mods folder.",
                    egui::FontId::proportional(20.0),
                    Color32::WHITE,
                );
                return;
            }
            
            // Handle mouse wheel (zoom)
            let scroll_delta = ui.input(|i| i.scroll_delta);
            if scroll_delta.y != 0.0 {
                let zoom_delta = scroll_delta.y * 0.001;
                self.data.zoom = (self.data.zoom + zoom_delta).max(0.1).min(5.0);
            }
            
            // Handle keyboard inputs
            if ui.input(|i| i.key_pressed(Key::F)) {
                // Reset view to fit graph
                self.fit_graph_to_view(rect.width(), rect.height());
            }

            if ui.input(|i| i.key_pressed(Key::Space)) {
                // Toggle selected node's position to be fixed/unfixed
                if let Some(node_idx) = self.data.drag_node {
                    // Toggle fixed state (would need to add a 'fixed' field to Node struct)
                    // self.data.nodes[node_idx].fixed = !self.data.nodes[node_idx].fixed;
                }
            }

            // Handle dragging (pan or move node)
            if response.drag_started() {
                self.data.dragging = true;
                self.data.drag_start = response.interact_pointer_pos().unwrap();
                self.data.drag_node = self.find_node_at_pos(self.data.drag_start);
            }
            
            if self.data.dragging && response.dragged() {
                let current_pos = response.interact_pointer_pos().unwrap();
                let delta = current_pos - self.data.drag_start;
                
                if let Some(node_idx) = self.data.drag_node {
                    // Move the selected node
                    let screen_pos = self.world_to_screen(self.data.nodes[node_idx].pos);
                    let new_screen_pos = screen_pos + delta;
                    self.data.nodes[node_idx].pos = self.screen_to_world(new_screen_pos);
                } else {
                    // Pan the view
                    self.data.pan_offset += delta;
                }
                
                self.data.drag_start = current_pos;
            }
            
            if response.drag_released() {
                self.data.dragging = false;
                self.data.drag_node = None;
            }
            
            // Handle clicks (select node)
            if response.clicked() {
                if let Some(pos) = response.interact_pointer_pos() {
                    if let Some(node_idx) = self.find_node_at_pos(pos) {
                        self.select_node(node_idx);
                    } else {
                        self.data.selected_node = None;
                        self.data.mod_info = None;
                    }
                }
            }
            
            // Update hovered node
            if let Some(pos) = response.hover_pos() {
                self.data.hovered_node = self.find_node_at_pos(pos);
            } else {
                self.data.hovered_node = None;
            }
            
            // Draw edges first (so they appear behind nodes)
            for (edge_idx, edge) in self.data.edges.iter().enumerate() {
                let from_idx = self.data.nodes.iter().position(|n| n.id == edge.from);
                let to_idx = self.data.nodes.iter().position(|n| n.id == edge.to);
                
                if let (Some(from_idx), Some(to_idx)) = (from_idx, to_idx) {
                    let from_node = &self.data.nodes[from_idx];
                    let to_node = &self.data.nodes[to_idx];
                    
                    let from_pos = self.world_to_screen(from_node.pos);
                    let to_pos = self.world_to_screen(to_node.pos);
                    
                    // Check if this edge is connected to hovered or selected node
                    let highlight = self.data.hovered_node.map_or(false, |idx| idx == from_idx || idx == to_idx) ||
                                    self.data.selected_node.map_or(false, |id| id == edge.from || id == edge.to);
                    
                    // Enhance edge based on state
                    let edge_color = if highlight {
                        // Brighten the edge color when connected to hovered/selected node
                        let c = edge.color;
                        Color32::from_rgb(
                            (c.r() as f32 * 1.5).min(255.0) as u8,
                            (c.g() as f32 * 1.5).min(255.0) as u8,
                            (c.b() as f32 * 1.5).min(255.0) as u8
                        )
                    } else {
                        edge.color
                    };
                    
                    let stroke_width = if highlight { 2.5 } else { 1.5 };
                    
                    // Direction vector
                    let dir = normalize_vec(Vec2::new(to_pos.x - from_pos.x, to_pos.y - from_pos.y));
                    
                    // Adjust start and end positions for square nodes
                    let from_radius = from_node.radius * self.data.zoom;
                    let to_radius = to_node.radius * self.data.zoom;
                    
                    // Calculate intersection with square border
                    let (start_x, start_y) = if dir.x.abs() > dir.y.abs() {
                        // Horizontal dominant direction
                        let x_offset = from_radius * (dir.x / dir.x.abs());
                        let y_offset = from_radius * (dir.y / dir.x.abs());
                        (from_pos.x + x_offset, from_pos.y + y_offset)
                    } else {
                        // Vertical dominant direction
                        let y_offset = from_radius * (dir.y / dir.y.abs());
                        let x_offset = from_radius * (dir.x / dir.y.abs());
                        (from_pos.x + x_offset, from_pos.y + y_offset)
                    };
                    
                    let (end_x, end_y) = if dir.x.abs() > dir.y.abs() {
                        // Horizontal dominant direction
                        let x_offset = to_radius * (dir.x / dir.x.abs());
                        let y_offset = to_radius * (dir.y / dir.x.abs());
                        (to_pos.x - x_offset, to_pos.y - y_offset)
                    } else {
                        // Vertical dominant direction
                        let y_offset = to_radius * (dir.y / dir.y.abs());
                        let x_offset = to_radius * (dir.x / dir.y.abs());
                        (to_pos.x - x_offset, to_pos.y - y_offset)
                    };
                    
                    let start = Pos2::new(start_x, start_y);
                    let end = Pos2::new(end_x, end_y);
                    
                    // Draw the edge with its color
                    painter.line_segment([start, end], Stroke::new(stroke_width, edge_color));
                    
                    // Draw arrowhead
                    let arrow_size = if highlight { 12.0 } else { 10.0 };
                    let arrow_angle = std::f32::consts::PI / 8.0;
                    
                    let rotated_dir1 = rotate_vec(dir, arrow_angle);
                    let rotated_dir2 = rotate_vec(dir, -arrow_angle);
                    
                    let arrow1 = Pos2::new(
                        end.x - rotated_dir1.x * arrow_size,
                        end.y - rotated_dir1.y * arrow_size
                    );
                    let arrow2 = Pos2::new(
                        end.x - rotated_dir2.x * arrow_size,
                        end.y - rotated_dir2.y * arrow_size
                    );
                    
                    painter.line_segment([end, arrow1], Stroke::new(stroke_width, edge_color));
                    painter.line_segment([end, arrow2], Stroke::new(stroke_width, edge_color));
                    
                    // Draw edge label
                    let mid_pos = Pos2::new(
                        (start.x + end.x) * 0.5,
                        (start.y + end.y) * 0.5
                    );
                    
                    // Include side info in the edge label if available
                    let label_text = if let Some(side) = &edge.side {
                        format!("{} [{}]", edge.label, side)
                    } else {
                        edge.label.clone()
                    };
                    
                    // Draw a small background for better readability
                    let text_size = egui::TextStyle::Body.resolve(ui.style()).size;
                    let text_width = label_text.len() as f32 * text_size * 0.6;
                    let bg_rect = Rect::from_center_size(
                        mid_pos, 
                        Vec2::new(text_width, text_size * 1.2)
                    );
                    painter.rect_filled(bg_rect, 3.0, Color32::from_rgba_premultiplied(0, 0, 0, 200));
                    
                    painter.text(
                        mid_pos,
                        egui::Align2::CENTER_CENTER,
                        &label_text,
                        egui::FontId::proportional(if highlight { 12.0 } else { 10.0 }),
                        Color32::WHITE,
                    );
                }
            }
            
            // Draw nodes
            for (i, node) in self.data.nodes.iter().enumerate() {
                let pos = self.world_to_screen(node.pos);
                let radius = node.radius * self.data.zoom;
                
                let is_selected = self.data.selected_node.map_or(false, |id| id == node.id);
                let is_hovered = self.data.hovered_node.map_or(false, |idx| idx == i);
                
                let node_color = if is_selected {
                    Color32::from_rgb(255, 150, 50)
                } else if is_hovered {
                    Color32::from_rgb(200, 200, 50)
                } else {
                    node.color
                };
                
                let stroke = if is_selected {
                    Stroke::new(2.5, Color32::WHITE)
                } else if is_hovered {
                    Stroke::new(2.0, Color32::LIGHT_YELLOW)
                } else {
                    Stroke::new(1.0, Color32::BLACK)
                };
                
                // Draw node background and shape
                if let Some(texture_id) = node.texture {
                    // Add a white background for transparent icons
                    let bg_rect = Rect::from_center_size(
                        pos, 
                        Vec2::new(radius * 2.0, radius * 2.0)
                    );
                    painter.rect_filled(bg_rect, 4.0, Color32::from_rgb(40, 40, 40));
                    
                    // Add a white background for the icon
                    let inner_rect = Rect::from_center_size(
                        pos, 
                        Vec2::new(radius * 1.8, radius * 1.8)
                    );
                    painter.rect_filled(inner_rect, 3.0, Color32::WHITE);
                    
                    // Calculate icon size to fit within the node
                    let icon_size = radius * 1.6;
                    
                    // Create a square for the icon centered on the node
                    let rect = Rect::from_center_size(
                        pos, 
                        Vec2::new(icon_size, icon_size)
                    );
                    
                    // Draw the icon
                    painter.image(texture_id, rect, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), Color32::WHITE);
                    
                    // Draw colored border around icon
                    painter.rect_stroke(bg_rect, 4.0, Stroke::new(3.0, node_color));
                } else {
                    // Draw regular filled square
                    let rect = Rect::from_center_size(
                        pos, 
                        Vec2::new(radius * 2.0, radius * 2.0)
                    );
                    painter.rect_filled(rect, 4.0, node_color);
                    
                    // Draw node outline
                    painter.rect_stroke(rect, 4.0, stroke);
                }
                
                // Draw node label below the node with improved readability
                let font_size = (12.0 * self.data.zoom).max(8.0).min(16.0);
                let text_pos = pos2(pos.x, pos.y + radius + 8.0);
                
                // Draw text background with better contrast
                let text_size = egui::TextStyle::Body.resolve(ui.style()).size * (font_size / 12.0);
                let text_width = node.label.len() as f32 * text_size * 0.55;
                let text_bg_rect = Rect::from_center_size(
                    text_pos,
                    Vec2::new(text_width, text_size * 1.2),
                );
                painter.rect_filled(text_bg_rect, 3.0, Color32::from_rgba_premultiplied(0, 0, 0, 200));
                
                painter.text(
                    text_pos,
                    egui::Align2::CENTER_CENTER,
                    &node.label,
                    egui::FontId::proportional(font_size),
                    Color32::WHITE,
                );
            }
        });
        
        // Status bar
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("Mods loaded: {}", self.data.mods.len()));
                
                if let Some(graph) = &self.data.graph {
                    ui.separator();
                    ui.label(format!("Dependencies: {}", graph.edge_count()));
                }
                
                ui.separator();
                ui.label(format!("Zoom: {:.1}x", self.data.zoom));
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label("Pan: drag background | Zoom: mouse wheel | F: fit view | Select: click node");
                });
            });
        });
    }
}

fn scan_mods(mods_path: &Path, tx: Sender<AppMessage>) -> Result<(Vec<(String, ModFormat)>, Vec<ModIcon>)> {
    let mut results = Vec::new();
    let mut icons = Vec::new();
    
    let total_files = WalkDir::new(mods_path)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| {
            let path = e.path();
            // Filter out .connector and .index folders
            if path.is_dir() {
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                return name != ".connector" && name != ".index";
            }
            path.extension().map_or(false, |ext| ext == "jar")
        })
        .count();
    
    let mut processed = 0;
    
    for entry in WalkDir::new(mods_path)
        .into_iter()
        .filter_map(Result::ok)
        .filter(|e| {
            let path = e.path();
            // Filter out .connector and .index folders
            if path.is_dir() {
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                return name != ".connector" && name != ".index";
            }
            true
        }) {
        let path = entry.path();
        
        // Only process JAR files
        if path.extension().map_or(false, |ext| ext == "jar") {
            let file_name = path.file_name().unwrap_or_default().to_string_lossy();
            tx.send(AppMessage::Progress(format!("Processing: {} ({}/{})", 
                file_name, processed + 1, total_files))).unwrap();
            
            match extract_mod_info_and_icon(path) {
                Ok((Some((jar_name, mod_format)), Some(icon))) => {
                    results.push((jar_name, mod_format));
                    icons.push(icon);
                }
                Ok((Some((jar_name, mod_format)), None)) => {
                    results.push((jar_name, mod_format));
                }
                Ok((None, Some(icon))) => {
                    icons.push(icon);
                }
                Ok((None, None)) => {
                    // No mod info found - skip silently
                }
                Err(e) => {
                    tx.send(AppMessage::Progress(format!("Error processing {}: {}", file_name, e))).unwrap();
                }
            }
            
            processed += 1;
            if processed % 10 == 0 || processed == total_files {
                tx.send(AppMessage::Progress(format!(
                    "Processed {}/{} mods", processed, total_files
                ))).unwrap();
            }
        }
    }
    
    Ok((results, icons))
}

fn extract_mod_info_and_icon(jar_path: &Path) -> Result<(Option<(String, ModFormat)>, Option<ModIcon>)> {
    let file = File::open(jar_path)?;
    let mut archive = ZipArchive::new(file)?;
    let jar_name = jar_path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .to_string();
    
    let mut mod_format = None;
    let mut icon_data = None;
    let mut mod_id = String::new();
    let mut icon_path = None;
    
    println!("Examining JAR: {}", jar_name);
    
    // List all files in the JAR for debugging
    let file_count = archive.len();
    println!("  - Contains {} files", file_count);
    
    // Collect all file names and possible icon files for later
    let mut all_file_names = Vec::new();
    for i in 0..file_count {
        if let Ok(file) = archive.by_index(i) {
            all_file_names.push(file.name().to_string());
        }
    }
    
    // Try to find mod information - avoid nested borrows
    let mut neoforge_toml_content = String::new();
    let mut fabric_json_content = String::new();
    
    // Check if files exist and read their contents
    let has_neoforge = archive.by_name("META-INF/neoforge.mods.toml").map(|mut file| {
        println!("  - Found neoforge.mods.toml");
        file.read_to_string(&mut neoforge_toml_content).is_ok()
    }).unwrap_or(false);
    
    let has_fabric = archive.by_name("fabric.mod.json").map(|mut file| {
        println!("  - Found fabric.mod.json");
        file.read_to_string(&mut fabric_json_content).is_ok()
    }).unwrap_or(false);
    
    // Process NeoForge first
    if has_neoforge {
        if let Ok(config) = toml::from_str::<ModToml>(&neoforge_toml_content) {
            // Extract mod ID for icon lookup
            if !config.mods.is_empty() {
                mod_id = config.mods[0].mod_id.clone();
                println!("  - NeoForge mod ID: {}", mod_id);
                
                // Extract icon path from TOML if available
                if let Some(logo_file) = config.mods[0].logo_file.as_ref() {
                    if !logo_file.is_empty() {
                        icon_path = Some(logo_file.clone());
                        println!("  - Found NeoForge icon path: {}", logo_file);
                    }
                }
            }
            
            mod_format = Some(ModFormat::NeoForge(config));
        }
    }
    // Then Fabric
    else if has_fabric {
        if let Ok(config) = serde_json::from_str::<FabricMod>(&fabric_json_content) {
            mod_id = config.id.clone();
            println!("  - Fabric mod ID: {}", mod_id);
            
            // Extract icon path from JSON if available
            if let Some(fabric_icon) = &config.icon {
                if !fabric_icon.is_empty() {
                    icon_path = Some(fabric_icon.clone());
                    println!("  - Found Fabric icon path: {}", fabric_icon);
                }
            }
            
            mod_format = Some(ModFormat::Fabric(config));
        }
    }
    
    // Try to load the icon
    if !mod_id.is_empty() {
        // Try the exact icon path first
        if let Some(path) = &icon_path {
            if try_read_icon_file(&mut archive, path, &mod_id, &mut icon_data) {
                return Ok((mod_format.map(|mf| (jar_name, mf)), icon_data));
            }
            
            // Try case-insensitive match
            for file_name in &all_file_names {
                if file_name.to_lowercase() == path.to_lowercase() && file_name != path {
                    println!("  - Found icon path with different case: {} vs {}", path, file_name);
                    if try_read_icon_file(&mut archive, file_name, &mod_id, &mut icon_data) {
                        return Ok((mod_format.map(|mf| (jar_name, mf)), icon_data));
                    }
                }
            }
        }
        
        // Try common icon paths
        println!("  - Trying common icon paths for mod: {}", mod_id);
        let icon_paths = vec![
            format!("assets/{}/icon.png", mod_id),
            format!("assets/{}/logo.png", mod_id),
            format!("assets/{}/icon.jpg", mod_id),
            "assets/vmp/icon.png".to_string(),
            format!("assets/{}/icon.png", mod_id.to_lowercase()),
            format!("{}.png", mod_id),
            format!("{}-icon.png", mod_id),
            format!("{}_icon.png", mod_id),
            format!("{}/icon.png", mod_id),
            format!("{}/logo.png", mod_id),
            "icon.png".to_string(),
            "logo.png".to_string(),
            "assets/icon.png".to_string(),
            "assets/logo.png".to_string(),
            "META-INF/icon.png".to_string(),
            "META-INF/logo.png".to_string(),
        ];
        
        for path in &icon_paths {
            if try_read_icon_file(&mut archive, path, &mod_id, &mut icon_data) {
                break;
            }
        }
        
        // Try scanning for potential icon files
        if icon_data.is_none() {
            println!("  - Scanning all files for potential icon...");
            
            // Filter and collect potential icon files
            let potential_icons: Vec<String> = all_file_names.iter()
                .filter(|name| {
                    let name_lower = name.to_lowercase();
                    (name_lower.ends_with(".png") || name_lower.ends_with(".jpg") || name_lower.ends_with(".jpeg"))
                        && (name_lower.contains("icon") || name_lower.contains("logo") || name_lower.contains(&mod_id.to_lowercase()))
                })
                .cloned()
                .collect();
            
            for file_name in potential_icons {
                println!("  - Found potential icon by scanning: {}", file_name);
                if try_read_icon_file(&mut archive, &file_name, &mod_id, &mut icon_data) {
                    break;
                }
            }
        }
    }
    
    Ok((mod_format.map(|mf| (jar_name, mf)), icon_data))
}

// Helper function to try reading an icon file from archive
fn try_read_icon_file(archive: &mut ZipArchive<File>, path: &str, mod_id: &str, icon_data: &mut Option<ModIcon>) -> bool {
    match archive.by_name(path) {
        Ok(mut file) => {
            let mut buffer = Vec::new();
            if file.read_to_end(&mut buffer).is_ok() {
                println!("  - Successfully loaded icon from path: {} (size: {} bytes)", path, buffer.len());
                *icon_data = Some(ModIcon {
                    mod_id: mod_id.to_string(),
                    image_data: buffer,
                });
                true
            } else {
                println!("  - Failed to read icon data from: {}", path);
                false
            }
        },
        Err(_) => {
            false
        }
    }
}

fn generate_dependency_graph(mods: &[(String, ModFormat)]) -> Result<DiGraph<String, String>> {
    let mut graph = DiGraph::new();
    let mut node_map = HashMap::new();
    let mut mod_id_to_jar = HashMap::new();
    let mut mod_environments = HashMap::new();
    let mut edge_info = Vec::new();
    
    // First pass: create nodes for each mod
    for (jar_name, mod_format) in mods {
        match mod_format {
            ModFormat::NeoForge(neoforge) => {
                for mod_entry in &neoforge.mods {
                    let display_name = if mod_entry.display_name.is_empty() {
                        format!("{} ({})", mod_entry.mod_id, mod_entry.version)
                    } else {
                        format!("{} ({})", mod_entry.display_name, mod_entry.version)
                    };
                    
                    let node_idx = graph.add_node(display_name);
                    node_map.insert(mod_entry.mod_id.clone(), node_idx);
                    mod_id_to_jar.insert(mod_entry.mod_id.clone(), jar_name.clone());
                }
            },
            ModFormat::Fabric(fabric) => {
                let display_name = if let Some(name) = &fabric.name {
                    format!("{} ({})", name, fabric.version)
                } else {
                    format!("{} ({})", fabric.id, fabric.version)
                };
                
                let node_idx = graph.add_node(display_name);
                node_map.insert(fabric.id.clone(), node_idx);
                mod_id_to_jar.insert(fabric.id.clone(), jar_name.clone());
                
                // Store environment info
                if let Some(env) = &fabric.environment {
                    mod_environments.insert(fabric.id.clone(), env.clone());
                }
            }
        }
    }
    
    // Debug output to find issues
    println!("Created {} nodes in graph", graph.node_count());
    
    // Second pass: add edges for dependencies
    for (_, mod_format) in mods {
        match mod_format {
            ModFormat::NeoForge(neoforge) => {
                for (base_mod_id, dependencies) in &neoforge.dependencies {
                    if let Some(from_idx) = node_map.get(base_mod_id) {
                        for dep in dependencies {
                            if let Some(to_idx) = node_map.get(&dep.mod_id) {
                                let edge_label = format!("{}", dep.dependency_type);
                                let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                                
                                // Store edge with side information
                                edge_info.push((edge_idx, dep.side.clone()));
                            }
                        }
                    }
                }
            },
            ModFormat::Fabric(fabric) => {
                if let Some(from_idx) = node_map.get(&fabric.id) {
                    // Get environment for this mod
                    let env = fabric.environment.as_ref().map(|e| e.as_str()).unwrap_or("*");
                    
                    // Add required dependencies
                    for (dep_id, _) in &fabric.depends {
                        if let Some(to_idx) = node_map.get(dep_id) {
                            let edge_label = "required".to_string();
                            let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                            
                            // Get dependency environment
                            let dep_env = mod_environments.get(dep_id).map(|e| e.as_str()).unwrap_or("*");
                            let side = determine_side_from_environments(env, dep_env);
                            
                            edge_info.push((edge_idx, Some(side)));
                        }
                    }
                    
                    // Add recommended dependencies
                    for (dep_id, _) in &fabric.recommends {
                        if let Some(to_idx) = node_map.get(dep_id) {
                            let edge_label = "recommends".to_string();
                            let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                            
                            // Get dependency environment
                            let dep_env = mod_environments.get(dep_id).map(|e| e.as_str()).unwrap_or("*");
                            let side = determine_side_from_environments(env, dep_env);
                            
                            edge_info.push((edge_idx, Some(side)));
                        }
                    }
                    
                    // Add optional dependencies
                    for (dep_id, _) in &fabric.suggests {
                        if let Some(to_idx) = node_map.get(dep_id) {
                            let edge_label = "suggests".to_string();
                            let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                            
                            // Get dependency environment
                            let dep_env = mod_environments.get(dep_id).map(|e| e.as_str()).unwrap_or("*");
                            let side = determine_side_from_environments(env, dep_env);
                            
                            edge_info.push((edge_idx, Some(side)));
                        }
                    }
                    
                    // Add conflicts
                    for (dep_id, _) in &fabric.conflicts {
                        if let Some(to_idx) = node_map.get(dep_id) {
                            let edge_label = "conflicts".to_string();
                            let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                            
                            // Get dependency environment
                            let dep_env = mod_environments.get(dep_id).map(|e| e.as_str()).unwrap_or("*");
                            let side = determine_side_from_environments(env, dep_env);
                            
                            edge_info.push((edge_idx, Some(side)));
                        }
                    }
                    
                    // Add breaks
                    for (dep_id, _) in &fabric.breaks {
                        if let Some(to_idx) = node_map.get(dep_id) {
                            let edge_label = "breaks".to_string();
                            let edge_idx = graph.add_edge(*from_idx, *to_idx, edge_label);
                            
                            // Get dependency environment
                            let dep_env = mod_environments.get(dep_id).map(|e| e.as_str()).unwrap_or("*");
                            let side = determine_side_from_environments(env, dep_env);
                            
                            edge_info.push((edge_idx, Some(side)));
                        }
                    }
                }
            }
        }
    }
    
    println!("Added {} edges in graph with {} edge info entries", graph.edge_count(), edge_info.len());
    
    // If no edges were found, add all nodes as isolated nodes
    if graph.edge_count() == 0 && graph.node_count() > 0 {
        println!("No edges found, adding isolated nodes");
    }
    
    Ok(graph)
}

// Helper function to determine side from environment settings
fn determine_side_from_environments(mod_env: &str, dep_env: &str) -> String {
    match (mod_env, dep_env) {
        ("client", _) => "CLIENT".to_string(),
        (_, "client") => "CLIENT".to_string(),
        ("server", _) => "SERVER".to_string(),
        (_, "server") => "SERVER".to_string(),
        _ => "BOTH".to_string(),
    }
}

fn load_image_from_memory(image_data: &[u8]) -> Result<DynamicImage> {
    println!("Loading image from memory, data size: {} bytes", image_data.len());
    
    let img = match image::load_from_memory(image_data) {
        Ok(img) => {
            println!("  - Image loaded successfully: {}x{}", img.width(), img.height());
            img
        },
        Err(e) => {
            println!("  - Failed to load image: {}", e);
            return Err(anyhow::anyhow!("Failed to load image: {}", e));
        }
    };
    
    Ok(img)
}

fn main() -> Result<()> {
    let args = Args::parse();
    
    println!("Starting mod dependency graph for directory: {:?}", args.mods_path);
    if !args.mods_path.exists() {
        println!("Warning: Mods path does not exist!");
    }
    
    let options = NativeOptions {
        initial_window_size: Some(Vec2::new(1280.0, 800.0)),
        ..Default::default()
    };
    
    eframe::run_native(
        "Minecraft Mod Dependency Graph",
        options,
        Box::new(|cc| Box::new(ModDepGraphApp::new(cc, args.mods_path)))
    ).map_err(|e| anyhow::anyhow!("Application error: {}", e))
}
