use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use bevy::{
    app::AppExit,
    picking::hover::Hovered,
    prelude::*,
    text::{FontSize, TextFont},
    ui::Pressed,
    ui_widgets::{Activate, Button, observe},
};
use serde::Deserialize;

use crate::{
    config::{RuntimeConfig, WorldLibrary},
    engine::AppMode,
    persistence::{WorldSaveTarget, WorldStateFile},
    physics::DestructionJournal,
};
use avian3d::prelude::Gravity;

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MenuState>()
            .add_systems(Startup, setup_menu)
            .add_systems(Startup, inspect_menu_runtime.after(setup_menu))
            .add_systems(
                Update,
                (
                    toggle_menu_with_escape,
                    update_button_styles,
                    sync_menu_visibility,
                ),
            );
    }
}

fn inspect_menu_runtime(
    roots: Query<&Visibility, With<MenuRoot>>,
    library_panels: Query<&Visibility, With<LibraryOnly>>,
    pause_panels: Query<&Visibility, With<SimulationOnly>>,
    buttons: Query<Entity, With<Button>>,
    cameras: Query<(Entity, &Camera), With<Camera3d>>,
) {
    info!(
        "Menu ECS probe: roots={}, library_panels={}, pause_panels={}, buttons={}, cameras={}",
        roots.iter().count(),
        library_panels.iter().count(),
        pause_panels.iter().count(),
        buttons.iter().count(),
        cameras.iter().count(),
    );
    for (entity, camera) in &cameras {
        info!(
            "Menu ECS probe: camera={entity:?}, active={}",
            camera.is_active
        );
    }
    for visibility in &roots {
        info!("Menu ECS probe: root visibility={visibility:?}");
    }
    for visibility in &library_panels {
        info!("Menu ECS probe: library visibility={visibility:?}");
    }
}

#[derive(Resource, Default)]
pub(crate) struct MenuState {
    pub(crate) visible: bool,
}

#[derive(Component)]
struct MenuRoot;

#[derive(Component)]
struct MenuAction(String);

#[derive(Component)]
struct LibraryOnly;

#[derive(Component)]
struct SimulationOnly;

#[derive(Component, Clone, Copy)]
struct MenuButtonColors {
    normal: Color,
    hovered: Color,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MenuAsset {
    schema_version: u32,
    title: String,
    subtitle: String,
    background_srgba: [f32; 4],
    panel_srgba: [f32; 4],
    title_srgba: [f32; 4],
    subtitle_srgba: [f32; 4],
    button_srgba: [f32; 4],
    button_hover_srgba: [f32; 4],
    button_text_srgba: [f32; 4],
    panel_width_px: f32,
    panel_padding_px: f32,
    vertical_gap_px: f32,
    title_size_px: f32,
    subtitle_size_px: f32,
    button_width_px: f32,
    button_height_px: f32,
    button_font_size_px: f32,
    #[serde(default = "default_world_card_height_px")]
    world_card_height_px: f32,
    #[serde(default = "default_world_card_gap_px")]
    world_card_gap_px: f32,
    #[serde(default = "default_world_preview_size_px")]
    world_preview_size_px: f32,
    #[serde(default = "default_world_description_size_px")]
    world_description_size_px: f32,
    #[serde(default = "default_world_metadata_size_px")]
    world_metadata_size_px: f32,
    #[serde(default = "default_world_empty_description")]
    world_empty_description_text: String,
    #[serde(default = "default_world_preview_label")]
    world_preview_fallback_label: String,
    #[serde(default = "default_world_version_prefix")]
    world_preview_version_prefix: String,
    #[serde(default = "default_world_format_label")]
    world_format_label: String,
    #[serde(default = "default_world_regions_label")]
    world_regions_label: String,
    #[serde(default = "default_world_objects_label")]
    world_objects_label: String,
    #[serde(default = "default_world_metadata_max_fields")]
    world_metadata_max_fields: usize,
    buttons: Vec<MenuButtonDefinition>,
}

fn default_world_card_height_px() -> f32 {
    116.0
}

fn default_world_card_gap_px() -> f32 {
    12.0
}

fn default_world_preview_size_px() -> f32 {
    84.0
}

fn default_world_description_size_px() -> f32 {
    11.0
}

fn default_world_metadata_size_px() -> f32 {
    9.0
}

fn default_world_empty_description() -> String {
    "No description supplied.".to_owned()
}

fn default_world_preview_label() -> String {
    "WORLD".to_owned()
}

fn default_world_version_prefix() -> String {
    "V".to_owned()
}

fn default_world_format_label() -> String {
    "FORMAT".to_owned()
}

fn default_world_regions_label() -> String {
    "REGIONS".to_owned()
}

fn default_world_objects_label() -> String {
    "OBJECTS".to_owned()
}

fn default_world_metadata_max_fields() -> usize {
    4
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MenuButtonDefinition {
    label: String,
    action: String,
}

impl MenuAsset {
    fn validate(&self) -> Result<(), String> {
        if !matches!(self.schema_version, 1 | 2) {
            return Err(format!(
                "unsupported menu schema version {}",
                self.schema_version
            ));
        }
        if self.title.trim().is_empty() || self.subtitle.trim().is_empty() {
            return Err("menu title and subtitle cannot be empty".to_owned());
        }
        let presentation_labels = [
            &self.world_empty_description_text,
            &self.world_preview_fallback_label,
            &self.world_preview_version_prefix,
            &self.world_format_label,
            &self.world_regions_label,
            &self.world_objects_label,
        ];
        if presentation_labels
            .iter()
            .any(|label| label.trim().is_empty())
        {
            return Err("world-library presentation labels cannot be empty".to_owned());
        }
        if self.world_metadata_max_fields == 0 {
            return Err("world_metadata_max_fields must be greater than zero".to_owned());
        }

        let colors = [
            self.background_srgba,
            self.panel_srgba,
            self.title_srgba,
            self.subtitle_srgba,
            self.button_srgba,
            self.button_hover_srgba,
            self.button_text_srgba,
        ];
        if !colors
            .iter()
            .flatten()
            .all(|value| value.is_finite() && (0.0..=1.0).contains(value))
        {
            return Err("menu colors must be finite sRGBA values in [0, 1]".to_owned());
        }

        let dimensions = [
            self.panel_width_px,
            self.panel_padding_px,
            self.vertical_gap_px,
            self.title_size_px,
            self.subtitle_size_px,
            self.button_width_px,
            self.button_height_px,
            self.button_font_size_px,
            self.world_card_height_px,
            self.world_card_gap_px,
            self.world_preview_size_px,
            self.world_description_size_px,
            self.world_metadata_size_px,
        ];
        if !dimensions
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
        {
            return Err("menu dimensions and font sizes must be positive and finite".to_owned());
        }
        if self.buttons.is_empty() {
            return Err("menu must define at least one button".to_owned());
        }

        for button in &self.buttons {
            if button.label.trim().is_empty() {
                return Err("menu button labels cannot be empty".to_owned());
            }
            if !matches!(button.action.as_str(), "start" | "resume" | "quit") {
                return Err(format!(
                    "unsupported menu action '{}' (expected start, resume, or quit)",
                    button.action
                ));
            }
        }
        Ok(())
    }
}

fn setup_menu(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<RuntimeConfig>,
    library: Res<WorldLibrary>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    let menu_path = config.paths.assets_dir.join("ui/main_menu.json");
    info!(
        "Initializing menu: world='{}', library_start={}, discovered_worlds={}, menu_asset='{}'",
        config.world.id,
        config.start_in_library,
        library.entries.len(),
        menu_path.display()
    );
    let menu = match load_menu(&menu_path) {
        Ok(menu) => menu,
        Err(error) => {
            error!(
                "Could not load menu presentation '{}': {error}.",
                menu_path.display()
            );
            return;
        }
    };

    state.visible = config.start_in_library;
    if state.visible {
        virtual_time.pause();
    } else {
        virtual_time.unpause();
    }

    let background_color = color_from_srgba(menu.background_srgba);
    let panel_color = color_from_srgba(menu.panel_srgba);
    let title_color = color_from_srgba(menu.title_srgba);
    let subtitle_color = color_from_srgba(menu.subtitle_srgba);
    let button_color = color_from_srgba(menu.button_srgba);
    let button_hover_color = color_from_srgba(menu.button_hover_srgba);
    let button_text_color = color_from_srgba(menu.button_text_srgba);
    let root_visibility = if state.visible {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    info!(
        "Menu asset loaded; scheduling UI entities with initial visibility {:?}",
        root_visibility
    );

    commands
        .spawn((
            MenuRoot,
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            },
            BackgroundColor(background_color),
            root_visibility,
        ))
        .with_children(|root| {
            root.spawn((
                LibraryOnly,
                Visibility::Visible,
                Node {
                    width: Val::Px(menu.panel_width_px),
                    max_height: Val::Percent(90.0),
                    padding: UiRect::all(Val::Px(menu.panel_padding_px)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    row_gap: Val::Px(menu.vertical_gap_px),
                    overflow: Overflow::clip_y(),
                    ..default()
                },
                BackgroundColor(panel_color),
            ))
            .with_children(|panel| {
                panel.spawn((
                    Text::new(menu.title.clone()),
                    TextFont {
                        font_size: FontSize::Px(menu.title_size_px),
                        ..default()
                    },
                    TextColor(title_color),
                ));
                panel.spawn((
                    Text::new(menu.subtitle.clone()),
                    TextFont {
                        font_size: FontSize::Px(menu.subtitle_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                if library.entries.is_empty() {
                    panel.spawn((
                        Text::new("No worlds were found under the project's worlds directory."),
                        TextFont {
                            font_size: FontSize::Px(menu.button_font_size_px),
                            ..default()
                        },
                        TextColor(subtitle_color),
                    ));
                }

                for world in &library.entries {
                    let preview = world
                        .preview_asset
                        .as_ref()
                        .map(|path| asset_server.load(path.clone()));
                    let schema_version = world.schema_version;
                    let preview_fallback_label = menu.world_preview_fallback_label.clone();
                    let preview_version_prefix = menu.world_preview_version_prefix.clone();
                    let description = if world.description.trim().is_empty() {
                        menu.world_empty_description_text.clone()
                    } else {
                        world.description.clone()
                    };
                    let metadata_lines =
                        world_metadata_lines(&world.metadata, menu.world_metadata_max_fields);
                    let format_summary = format!(
                        "{} v{}  |  {} {}  |  {} {}",
                        menu.world_format_label,
                        world.schema_version,
                        world.region_count,
                        menu.world_regions_label,
                        world.object_count,
                        menu.world_objects_label
                    );

                    panel
                        .spawn((
                            Button,
                            Hovered::default(),
                            Node {
                                width: Val::Px(menu.button_width_px),
                                min_height: Val::Px(menu.world_card_height_px),
                                padding: UiRect::all(Val::Px(12.0)),
                                flex_direction: FlexDirection::Row,
                                column_gap: Val::Px(menu.world_card_gap_px),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::FlexStart,
                                ..default()
                            },
                            BackgroundColor(button_color),
                            WorldSelection(world.manifest_path.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                            },
                            observe(handle_world_selection),
                        ))
                        .with_children(move |button| {
                            if let Some(preview) = preview.clone() {
                                button.spawn((
                                    ImageNode::new(preview),
                                    Node {
                                        width: Val::Px(menu.world_preview_size_px),
                                        height: Val::Px(menu.world_preview_size_px),
                                        flex_shrink: 0.0,
                                        ..default()
                                    },
                                ));
                            } else {
                                button
                                    .spawn((
                                        Node {
                                            width: Val::Px(menu.world_preview_size_px),
                                            height: Val::Px(menu.world_preview_size_px),
                                            flex_shrink: 0.0,
                                            align_items: AlignItems::Center,
                                            justify_content: JustifyContent::Center,
                                            ..default()
                                        },
                                        BackgroundColor(panel_color),
                                    ))
                                    .with_children(|tile| {
                                        tile.spawn((
                                            Text::new(format!(
                                                "{}\n{}{}",
                                                preview_fallback_label,
                                                preview_version_prefix,
                                                schema_version
                                            )),
                                            TextFont {
                                                font_size: FontSize::Px(
                                                    menu.world_metadata_size_px,
                                                ),
                                                ..default()
                                            },
                                            TextColor(subtitle_color),
                                        ));
                                    });
                            }

                            button
                                .spawn(Node {
                                    width: Val::Px(0.0),
                                    min_width: Val::Px(0.0),
                                    flex_grow: 1.0,
                                    flex_direction: FlexDirection::Column,
                                    justify_content: JustifyContent::Center,
                                    row_gap: Val::Px(4.0),
                                    ..default()
                                })
                                .with_children(move |details| {
                                    details.spawn((
                                        Text::new(format!("{} / {}", world.name, world.id)),
                                        TextFont {
                                            font_size: FontSize::Px(menu.button_font_size_px),
                                            ..default()
                                        },
                                        TextColor(button_text_color),
                                    ));
                                    details.spawn((
                                        Text::new(description.clone()),
                                        TextFont {
                                            font_size: FontSize::Px(menu.world_description_size_px),
                                            ..default()
                                        },
                                        TextColor(subtitle_color),
                                    ));
                                    details.spawn((
                                        Text::new(format_summary.clone()),
                                        TextFont {
                                            font_size: FontSize::Px(menu.world_metadata_size_px),
                                            ..default()
                                        },
                                        TextColor(subtitle_color),
                                    ));
                                    for metadata_line in &metadata_lines {
                                        details.spawn((
                                            Text::new(metadata_line.clone()),
                                            TextFont {
                                                font_size: FontSize::Px(
                                                    menu.world_metadata_size_px,
                                                ),
                                                ..default()
                                            },
                                            TextColor(subtitle_color),
                                        ));
                                    }
                                });
                        });
                }

                if let Some(quit) = menu.buttons.iter().find(|button| button.action == "quit") {
                    panel
                        .spawn((
                            Button,
                            Hovered::default(),
                            Node {
                                width: Val::Px(menu.button_width_px),
                                height: Val::Px(menu.button_height_px),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::Center,
                                ..default()
                            },
                            BackgroundColor(button_color),
                            MenuAction(quit.action.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                            },
                            observe(handle_menu_action),
                        ))
                        .with_children(|button| {
                            button.spawn((
                                Text::new(quit.label.clone()),
                                TextFont {
                                    font_size: FontSize::Px(menu.button_font_size_px),
                                    ..default()
                                },
                                TextColor(button_text_color),
                            ));
                        });
                }
            });

            root.spawn((
                SimulationOnly,
                Visibility::Hidden,
                Node {
                    width: Val::Px(menu.panel_width_px),
                    padding: UiRect::all(Val::Px(menu.panel_padding_px)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    row_gap: Val::Px(menu.vertical_gap_px),
                    ..default()
                },
                BackgroundColor(panel_color),
            ))
            .with_children(|panel| {
                panel.spawn((
                    Text::new("SIMULATION PAUSED"),
                    TextFont {
                        font_size: FontSize::Px(menu.title_size_px),
                        ..default()
                    },
                    TextColor(title_color),
                ));
                panel.spawn((
                    Text::new("Press Escape or choose Resume to continue"),
                    TextFont {
                        font_size: FontSize::Px(menu.subtitle_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                for definition in menu
                    .buttons
                    .iter()
                    .filter(|button| matches!(button.action.as_str(), "resume" | "quit"))
                {
                    panel
                        .spawn((
                            Button,
                            Hovered::default(),
                            Node {
                                width: Val::Px(menu.button_width_px),
                                height: Val::Px(menu.button_height_px),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::Center,
                                ..default()
                            },
                            BackgroundColor(button_color),
                            MenuAction(definition.action.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                            },
                            observe(handle_menu_action),
                        ))
                        .with_children(|button| {
                            button.spawn((
                                Text::new(definition.label.clone()),
                                TextFont {
                                    font_size: FontSize::Px(menu.button_font_size_px),
                                    ..default()
                                },
                                TextColor(button_text_color),
                            ));
                        });
                }
            });
        });
}

#[derive(Component)]
struct WorldSelection(PathBuf);

// This observer updates several Bevy resources as one atomic world-selection action.
#[allow(clippy::too_many_arguments)]
fn handle_world_selection(
    activate: On<Activate>,
    selections: Query<&WorldSelection>,
    mode: Res<State<AppMode>>,
    mut config: ResMut<RuntimeConfig>,
    mut world_state: ResMut<WorldStateFile>,
    mut save_target: ResMut<WorldSaveTarget>,
    mut journal: ResMut<DestructionJournal>,
    mut gravity: ResMut<Gravity>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut menu_state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if *mode.get() != AppMode::WorldLibrary {
        return;
    }

    let Ok(selection) = selections.get(activate.entity) else {
        return;
    };

    let data_dir = config.paths.data_dir.clone();
    let assets_dir = config.paths.assets_dir.clone();
    let project_root = config.paths.project_root.clone();
    let mut selected = match RuntimeConfig::load(data_dir, assets_dir, selection.0.clone()) {
        Ok(config) => config,
        Err(error) => {
            error!(
                "Could not load selected world '{}': {error:#}",
                selection.0.display()
            );
            return;
        }
    };
    selected.paths.project_root = project_root;

    let selected_state =
        match WorldStateFile::load_or_default(&selected.paths.world_state, &selected.world.id) {
            Ok(state) => state,
            Err(error) => {
                error!(
                    "Could not load save state for world '{}': {error:#}",
                    selected.world.id
                );
                return;
            }
        };

    gravity.0 = Vec3::from_array(
        selected
            .world
            .gravity_m_s2
            .unwrap_or(selected.engine.physics.gravity_m_s2),
    );
    save_target.world_id.clone_from(&selected.world.id);
    save_target
        .state_path
        .clone_from(&selected.paths.world_state);
    journal.fractured.clone_from(&selected_state.fractured);
    journal
        .broken_bonds
        .clone_from(&selected_state.broken_bonds);
    journal
        .damaged_bonds
        .clone_from(&selected_state.damaged_bonds);
    *world_state = selected_state;
    *config = selected;

    menu_state.visible = false;
    virtual_time.unpause();
    next_state.set(AppMode::Simulation);
    info!(
        "Selected world '{}'. Starting simulation.",
        config.world.name
    );
}

fn load_menu(path: &Path) -> Result<MenuAsset, String> {
    let source =
        std::fs::read_to_string(path).map_err(|error| format!("failed to read file: {error}"))?;
    let menu: MenuAsset =
        serde_json::from_str(&source).map_err(|error| format!("invalid menu JSON: {error}"))?;
    menu.validate()?;
    Ok(menu)
}

fn handle_menu_action(
    activate: On<Activate>,
    actions: Query<&MenuAction>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };

    match action.0.as_str() {
        "start" | "resume" => {
            state.visible = false;
            virtual_time.unpause();
        }
        "quit" => {
            app_exit.write(AppExit::Success);
        }
        _ => unreachable!("menu actions are validated during asset loading"),
    }
}

fn toggle_menu_with_escape(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<AppMode>>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if *mode.get() == AppMode::WorldLibrary || !keys.just_pressed(KeyCode::Escape) {
        return;
    }

    state.visible = !state.visible;
    if state.visible {
        virtual_time.pause();
    } else {
        virtual_time.unpause();
    }
}

fn update_button_styles(
    mut buttons: Query<
        (
            &Hovered,
            Has<Pressed>,
            &MenuButtonColors,
            &mut BackgroundColor,
        ),
        With<Button>,
    >,
) {
    for (hovered, pressed, colors, mut background) in &mut buttons {
        background.0 = if pressed || hovered.get() {
            colors.hovered
        } else {
            colors.normal
        };
    }
}

type RootVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<MenuRoot>,
        Without<LibraryOnly>,
        Without<SimulationOnly>,
    ),
>;
type LibraryVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<LibraryOnly>,
        Without<SimulationOnly>,
        Without<MenuRoot>,
    ),
>;
type SimulationVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<SimulationOnly>,
        Without<LibraryOnly>,
        Without<MenuRoot>,
    ),
>;

fn sync_menu_visibility(
    state: Res<MenuState>,
    mode: Res<State<AppMode>>,
    mut roots: RootVisibilityQuery<'_, '_>,
    mut library_panels: LibraryVisibilityQuery<'_, '_>,
    mut simulation_panels: SimulationVisibilityQuery<'_, '_>,
) {
    let (root_visibility, library_visibility, simulation_visibility) =
        menu_visibility_targets(*mode.get() == AppMode::WorldLibrary, state.visible);
    if state.is_changed() || mode.is_changed() {
        info!(
            "Menu state changed: mode={:?}, menu_visible={}, root={:?}, library={:?}, pause={:?}",
            mode.get(),
            state.visible,
            root_visibility,
            library_visibility,
            simulation_visibility
        );
    }

    // Evaluate the complete state every frame. This keeps the top-level overlay and its
    // explicitly-visible children in sync even when a state transition and activation occur
    // in the same frame.
    for mut visibility in &mut roots {
        visibility.set_if_neq(root_visibility);
    }
    for mut visibility in &mut library_panels {
        visibility.set_if_neq(library_visibility);
    }
    for mut visibility in &mut simulation_panels {
        visibility.set_if_neq(simulation_visibility);
    }
}

fn menu_visibility_targets(
    in_library: bool,
    menu_visible: bool,
) -> (Visibility, Visibility, Visibility) {
    let root = if in_library || menu_visible {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let library = if in_library {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let simulation = if !in_library && menu_visible {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    (root, library, simulation)
}

fn world_metadata_lines(
    metadata: &BTreeMap<String, serde_json::Value>,
    maximum_fields: usize,
) -> Vec<String> {
    metadata
        .iter()
        .take(maximum_fields)
        .filter_map(|(key, value)| {
            let display_value = metadata_value_to_display(value);
            (!display_value.is_empty())
                .then(|| format!("{}: {display_value}", key.replace('_', " ")))
        })
        .collect()
}

fn metadata_value_to_display(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::Null => String::new(),
        serde_json::Value::Bool(_) | serde_json::Value::Number(_) => value.to_string(),
        serde_json::Value::String(value) => value.clone(),
        serde_json::Value::Array(values) => values
            .iter()
            .map(metadata_value_to_display)
            .filter(|value| !value.is_empty())
            .collect::<Vec<_>>()
            .join(", "),
        serde_json::Value::Object(values) => values
            .iter()
            .map(|(key, value)| {
                let display_value = metadata_value_to_display(value);
                if display_value.is_empty() {
                    key.clone()
                } else {
                    format!("{key}: {display_value}")
                }
            })
            .collect::<Vec<_>>()
            .join(", "),
    }
}

fn color_from_srgba(value: [f32; 4]) -> Color {
    Color::srgba(value[0], value[1], value[2], value[3])
}

#[cfg(test)]
mod tests {
    use super::{
        MenuAsset, MenuButtonDefinition, default_world_empty_description,
        default_world_format_label, default_world_metadata_max_fields, default_world_objects_label,
        default_world_preview_label, default_world_regions_label, default_world_version_prefix,
        menu_visibility_targets, world_metadata_lines,
    };
    use bevy::prelude::Visibility;
    use std::collections::BTreeMap;

    fn valid_menu() -> MenuAsset {
        MenuAsset {
            schema_version: 1,
            title: "Title".to_owned(),
            subtitle: "Subtitle".to_owned(),
            background_srgba: [0.0, 0.0, 0.0, 1.0],
            panel_srgba: [0.1, 0.1, 0.1, 1.0],
            title_srgba: [1.0; 4],
            subtitle_srgba: [1.0; 4],
            button_srgba: [0.1, 0.1, 0.1, 1.0],
            button_hover_srgba: [0.2, 0.2, 0.2, 1.0],
            button_text_srgba: [1.0; 4],
            panel_width_px: 500.0,
            panel_padding_px: 32.0,
            vertical_gap_px: 12.0,
            title_size_px: 32.0,
            subtitle_size_px: 12.0,
            button_width_px: 300.0,
            button_height_px: 48.0,
            button_font_size_px: 16.0,
            world_card_height_px: 116.0,
            world_card_gap_px: 12.0,
            world_preview_size_px: 84.0,
            world_description_size_px: 11.0,
            world_metadata_size_px: 9.0,
            world_empty_description_text: default_world_empty_description(),
            world_preview_fallback_label: default_world_preview_label(),
            world_preview_version_prefix: default_world_version_prefix(),
            world_format_label: default_world_format_label(),
            world_regions_label: default_world_regions_label(),
            world_objects_label: default_world_objects_label(),
            world_metadata_max_fields: default_world_metadata_max_fields(),
            buttons: vec![MenuButtonDefinition {
                label: "Start".to_owned(),
                action: "start".to_owned(),
            }],
        }
    }

    #[test]
    fn world_library_visibility_is_independent_of_the_pause_menu_flag() {
        assert_eq!(
            menu_visibility_targets(true, false),
            (Visibility::Visible, Visibility::Visible, Visibility::Hidden)
        );
    }

    #[test]
    fn selecting_a_world_hides_both_the_overlay_and_library_panel() {
        assert_eq!(
            menu_visibility_targets(false, false),
            (Visibility::Hidden, Visibility::Hidden, Visibility::Hidden)
        );
    }

    #[test]
    fn simulation_pause_shows_only_the_pause_panel() {
        assert_eq!(
            menu_visibility_targets(false, true),
            (Visibility::Visible, Visibility::Hidden, Visibility::Visible)
        );
    }

    #[test]
    fn legacy_schema_one_menu_defaults_new_library_presentation_fields() {
        let legacy = serde_json::json!({
            "schema_version": 1,
            "title": "Legacy menu",
            "subtitle": "Legacy subtitle",
            "background_srgba": [0.0, 0.0, 0.0, 1.0],
            "panel_srgba": [0.1, 0.1, 0.1, 1.0],
            "title_srgba": [1.0, 1.0, 1.0, 1.0],
            "subtitle_srgba": [1.0, 1.0, 1.0, 1.0],
            "button_srgba": [0.1, 0.1, 0.1, 1.0],
            "button_hover_srgba": [0.2, 0.2, 0.2, 1.0],
            "button_text_srgba": [1.0, 1.0, 1.0, 1.0],
            "panel_width_px": 500.0,
            "panel_padding_px": 32.0,
            "vertical_gap_px": 12.0,
            "title_size_px": 32.0,
            "subtitle_size_px": 12.0,
            "button_width_px": 300.0,
            "button_height_px": 48.0,
            "button_font_size_px": 16.0,
            "buttons": [{"label": "QUIT", "action": "quit"}]
        });

        let menu: MenuAsset = serde_json::from_value(legacy).unwrap();
        assert!(menu.validate().is_ok());
        assert_eq!(menu.world_card_height_px, 116.0);
        assert_eq!(menu.world_preview_fallback_label, "WORLD");
        assert_eq!(menu.world_metadata_max_fields, 4);
    }

    #[test]
    fn world_metadata_arrays_render_as_compact_readable_rows() {
        let metadata = BTreeMap::from([
            ("author".to_owned(), serde_json::json!("sample content")),
            (
                "features".to_owned(),
                serde_json::json!(["rigid bodies", "bonded structures", "chunk streaming"]),
            ),
        ]);

        assert_eq!(
            world_metadata_lines(&metadata, 2),
            [
                "author: sample content",
                "features: rigid bodies, bonded structures, chunk streaming"
            ]
        );
        assert_eq!(world_metadata_lines(&metadata, 1).len(), 1);
    }

    #[test]
    fn menu_asset_rejects_unknown_actions_and_invalid_dimensions() {
        let mut menu = valid_menu();
        assert!(menu.validate().is_ok());

        menu.buttons[0].action = "spawn_admin".to_owned();
        assert!(menu.validate().is_err());

        let mut menu = valid_menu();
        menu.button_width_px = f32::NAN;
        assert!(menu.validate().is_err());
    }
}
