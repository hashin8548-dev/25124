use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use bevy::{
    app::AppExit,
    input::mouse::{MouseScrollUnit, MouseWheel},
    input_focus::{
        FocusCause, InputFocus,
        tab_navigation::{TabGroup, TabIndex, TabNavigationPlugin},
    },
    math::DVec3,
    picking::hover::{HoverMap, Hovered},
    prelude::*,
    text::{FontSize, TextFont},
    ui::Pressed,
    ui_widgets::{Activate, Button, observe},
};
use serde::{Deserialize, Serialize};

use crate::{
    config::{RuntimeConfig, WorldLibrary},
    engine::AppMode,
    persistence::{WorldSaveTarget, WorldStateFile},
    physics::DestructionJournal,
    world::WorldOrigin,
};
use avian3d::prelude::Gravity;

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(TabNavigationPlugin)
            .init_resource::<MenuState>()
            .init_resource::<MenuFeedback>()
            .init_resource::<QuitConfirmationState>()
            .add_systems(Startup, setup_menu)
            .add_systems(
                Update,
                (
                    toggle_menu_with_escape,
                    update_button_styles,
                    sync_menu_visibility,
                    update_world_list_scroll,
                    sync_menu_feedback,
                    sync_settings_labels,
                    sync_quit_confirmation_visibility,
                ),
            );
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum MenuPage {
    #[default]
    Home,
    Worlds,
    Settings,
}

#[derive(Resource, Default)]
pub(crate) struct MenuState {
    pub(crate) visible: bool,
    page: MenuPage,
    preferences: UserPreferences,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum MenuFeedbackTone {
    #[default]
    Notice,
    Success,
}

#[derive(Resource, Default)]
pub(crate) struct MenuFeedback {
    pub(crate) message: String,
    pub(crate) tone: MenuFeedbackTone,
}

#[derive(Resource, Default)]
struct QuitConfirmationState {
    visible: bool,
    return_focus: Option<Entity>,
}

#[derive(Component)]
struct MenuRoot;

#[derive(Component)]
struct WorldList;

#[derive(Component)]
struct MenuNotice;

#[derive(Component, Clone, Copy)]
struct MenuNoticeColors {
    notice: Color,
    success: Color,
}

#[derive(Component)]
struct QuitConfirmationPanel;

#[derive(Component)]
struct MenuAction(String);

#[derive(Component)]
struct HomeOnly;

#[derive(Component)]
struct MenuCursorMarker(Entity);

#[derive(Component)]
struct LibraryOnly;

#[derive(Component)]
struct SettingsOnly;

#[derive(Component)]
struct SimulationOnly;

#[derive(Component)]
struct SettingAction(String);

#[derive(Component)]
struct SettingsRenderDistanceValue;

#[derive(Component)]
struct SettingsLodDetailValue;

#[derive(Component, Default)]
struct MenuGlowState(bool);

#[derive(Component, Clone, Copy)]
struct MenuButtonColors {
    normal: Color,
    hovered: Color,
    border_normal: Color,
    border_active: Color,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MenuAsset {
    schema_version: u32,
    title: String,
    subtitle: String,
    #[serde(default = "default_home_buttons")]
    home_buttons: Vec<MenuButtonDefinition>,
    #[serde(default)]
    home_layout: HomeLayoutDefinition,
    #[serde(default = "default_empty_library_title")]
    empty_library_title: String,
    #[serde(default = "default_empty_library_hint")]
    empty_library_hint: String,
    #[serde(default = "default_quit_confirm_title")]
    quit_confirm_title: String,
    #[serde(default = "default_quit_confirm_message")]
    quit_confirm_message: String,
    #[serde(default = "default_quit_confirm_label")]
    quit_confirm_label: String,
    #[serde(default = "default_quit_cancel_label")]
    quit_cancel_label: String,
    #[serde(default = "default_library_navigation_hint")]
    library_navigation_hint: String,
    #[serde(default = "default_pause_controls_hint")]
    pause_controls_hint: String,
    #[serde(default = "default_pause_title")]
    pause_title: String,
    #[serde(default = "default_world_count_template")]
    world_count_template: String,
    #[serde(default = "default_library_warning_template")]
    library_warning_template: String,
    #[serde(default = "default_modal_backdrop_srgba")]
    modal_backdrop_srgba: [f32; 4],
    #[serde(default = "default_notice_srgba")]
    notice_srgba: [f32; 4],
    #[serde(default = "default_success_srgba")]
    success_srgba: [f32; 4],
    #[serde(default = "default_library_warning_srgba")]
    library_warning_srgba: [f32; 4],
    background_srgba: [f32; 4],
    panel_srgba: [f32; 4],
    title_srgba: [f32; 4],
    subtitle_srgba: [f32; 4],
    button_srgba: [f32; 4],
    button_hover_srgba: [f32; 4],
    button_text_srgba: [f32; 4],
    #[serde(default = "default_panel_border_srgba")]
    panel_border_srgba: [f32; 4],
    #[serde(default = "default_panel_shadow_srgba")]
    panel_shadow_srgba: [f32; 4],
    #[serde(default = "default_panel_shadow_blur_px")]
    panel_shadow_blur_px: f32,
    #[serde(default = "default_panel_shadow_offset_y_px")]
    panel_shadow_offset_y_px: f32,
    #[serde(default = "default_button_border_srgba")]
    button_border_srgba: [f32; 4],
    #[serde(default = "default_button_active_border_srgba")]
    button_active_border_srgba: [f32; 4],
    #[serde(default = "default_danger_button_srgba")]
    danger_button_srgba: [f32; 4],
    #[serde(default = "default_danger_button_hover_srgba")]
    danger_button_hover_srgba: [f32; 4],
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
    #[serde(default = "default_panel_radius_px")]
    panel_radius_px: f32,
    #[serde(default = "default_card_radius_px")]
    card_radius_px: f32,
    buttons: Vec<MenuButtonDefinition>,
}

fn default_empty_library_title() -> String {
    "YOUR WORLD LIBRARY IS EMPTY".to_owned()
}

fn default_empty_library_hint() -> String {
    "Add a world manifest at worlds/<name>/world.json to get started.".to_owned()
}

fn default_quit_confirm_title() -> String {
    "UNSAVED SIMULATION".to_owned()
}

fn default_quit_confirm_message() -> String {
    "Changes since your last F5 save will be lost. Choose Keep Playing to resume and save before quitting."
        .to_owned()
}

fn default_quit_confirm_label() -> String {
    "QUIT ANYWAY".to_owned()
}

fn default_quit_cancel_label() -> String {
    "KEEP PLAYING".to_owned()
}

fn default_library_navigation_hint() -> String {
    "TAB / SHIFT+TAB NAVIGATE   ·   ENTER SELECT".to_owned()
}

fn default_pause_controls_hint() -> String {
    "HOLD RMB TO LOOK  ·  WASD MOVE  ·  SHIFT SPRINT  ·  SPACE/CTRL VERTICAL\nTAB NAVIGATE  ·  ENTER ACTIVATE  ·  ESC RESUME  ·  F5 SAVE".to_owned()
}

fn default_pause_title() -> String {
    "SIMULATION PAUSED".to_owned()
}

fn default_world_count_template() -> String {
    "{count} WORLDS AVAILABLE".to_owned()
}

fn default_library_warning_template() -> String {
    "{count} LIBRARY NOTICE(S) — Some entries or previews were skipped. See the application log."
        .to_owned()
}

fn default_modal_backdrop_srgba() -> [f32; 4] {
    [0.004, 0.008, 0.016, 0.88]
}

fn default_notice_srgba() -> [f32; 4] {
    [0.95, 0.68, 0.32, 1.0]
}

fn default_success_srgba() -> [f32; 4] {
    [0.28, 0.78, 0.65, 1.0]
}

fn default_library_warning_srgba() -> [f32; 4] {
    [0.94, 0.70, 0.34, 1.0]
}

fn default_panel_border_srgba() -> [f32; 4] {
    [0.18, 0.34, 0.39, 0.72]
}

fn default_panel_shadow_srgba() -> [f32; 4] {
    [0.0, 0.0, 0.0, 0.48]
}

fn default_panel_shadow_blur_px() -> f32 {
    30.0
}

fn default_panel_shadow_offset_y_px() -> f32 {
    12.0
}

fn default_button_border_srgba() -> [f32; 4] {
    [0.16, 0.31, 0.36, 0.78]
}

fn default_button_active_border_srgba() -> [f32; 4] {
    [0.28, 0.78, 0.77, 1.0]
}

fn default_danger_button_srgba() -> [f32; 4] {
    [0.31, 0.075, 0.095, 1.0]
}

fn default_danger_button_hover_srgba() -> [f32; 4] {
    [0.48, 0.105, 0.14, 1.0]
}

fn default_panel_radius_px() -> f32 {
    18.0
}

fn default_card_radius_px() -> f32 {
    12.0
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

fn default_home_buttons() -> Vec<MenuButtonDefinition> {
    vec![
        MenuButtonDefinition {
            label: "CONTINUE".to_owned(),
            action: "continue".to_owned(),
        },
        MenuButtonDefinition {
            label: "NEW WORLD".to_owned(),
            action: "create_world".to_owned(),
        },
        MenuButtonDefinition {
            label: "WORLDS".to_owned(),
            action: "worlds".to_owned(),
        },
        MenuButtonDefinition {
            label: "SETTINGS".to_owned(),
            action: "settings".to_owned(),
        },
    ]
}

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct HomeLayoutDefinition {
    left_percent: f32,
    top_percent: f32,
    panel_width_px: f32,
    panel_height_percent: f32,
    title_frame_width_px: f32,
    title_frame_height_px: f32,
    title_font_size_px: f32,
    eyebrow_font_size_px: f32,
    version_font_size_px: f32,
    actions_top_gap_px: f32,
    action_list_gap_px: f32,
    action_row_width_px: f32,
    action_row_height_px: f32,
    action_button_width_px: f32,
    action_button_height_px: f32,
    action_button_left_px: f32,
    action_button_padding_left_px: f32,
    action_font_size_px: f32,
    cursor_font_size_px: f32,
}

impl Default for HomeLayoutDefinition {
    fn default() -> Self {
        Self {
            left_percent: 3.125,
            top_percent: 10.9,
            panel_width_px: 600.0,
            panel_height_percent: 86.0,
            title_frame_width_px: 540.0,
            title_frame_height_px: 190.0,
            title_font_size_px: 104.0,
            eyebrow_font_size_px: 16.0,
            version_font_size_px: 18.0,
            actions_top_gap_px: 82.0,
            action_list_gap_px: 16.0,
            action_row_width_px: 540.0,
            action_row_height_px: 76.0,
            action_button_width_px: 465.0,
            action_button_height_px: 76.0,
            action_button_left_px: 36.0,
            action_button_padding_left_px: 48.0,
            action_font_size_px: 32.0,
            cursor_font_size_px: 27.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MenuButtonDefinition {
    label: String,
    action: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
struct UserPreferences {
    schema_version: u32,
    last_world_manifest: Option<String>,
    render_distance_chunks: u32,
    /// 0 = performance, 1 = balanced, 2 = quality.
    lod_detail: u8,
}

impl Default for UserPreferences {
    fn default() -> Self {
        Self {
            schema_version: 1,
            last_world_manifest: None,
            render_distance_chunks: 2,
            lod_detail: 1,
        }
    }
}

fn user_preferences_path(project_root: &Path) -> PathBuf {
    project_root.join(".25124-preferences.json")
}

fn load_user_preferences(project_root: &Path) -> UserPreferences {
    let path = user_preferences_path(project_root);
    let source = match fs::read_to_string(&path) {
        Ok(source) => source,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            return UserPreferences::default();
        }
        Err(error) => {
            warn!("Could not read preferences '{}': {error}", path.display());
            return UserPreferences::default();
        }
    };

    let preferences: UserPreferences = match serde_json::from_str(&source) {
        Ok(preferences) => preferences,
        Err(error) => {
            warn!(
                "Preferences '{}' are invalid: {error}; using defaults",
                path.display()
            );
            return UserPreferences::default();
        }
    };
    if preferences.schema_version != 1
        || !(2..=8).contains(&preferences.render_distance_chunks)
        || preferences.lod_detail > 2
    {
        warn!(
            "Preferences '{}' contain unsupported or out-of-range values; using defaults",
            path.display()
        );
        return UserPreferences::default();
    }
    preferences
}

fn save_user_preferences(project_root: &Path, preferences: &UserPreferences) -> Result<(), String> {
    let path = user_preferences_path(project_root);
    let temporary_path = path.with_extension("json.tmp");
    let contents = serde_json::to_vec_pretty(preferences)
        .map_err(|error| format!("could not serialize preferences: {error}"))?;
    fs::write(&temporary_path, contents)
        .map_err(|error| format!("could not write preferences: {error}"))?;
    if let Err(first_error) = fs::rename(&temporary_path, &path) {
        // Windows does not replace an existing destination with rename; preserve the
        // temporary file until the previous preferences file has been removed.
        if path.exists() {
            fs::remove_file(&path).map_err(|error| {
                format!("could not replace preferences after '{first_error}': {error}")
            })?;
            fs::rename(&temporary_path, &path)
                .map_err(|error| format!("could not commit preferences: {error}"))?;
        } else {
            let _ = fs::remove_file(&temporary_path);
            return Err(format!("could not commit preferences: {first_error}"));
        }
    }
    Ok(())
}

fn preferred_world_manifest(
    project_root: &Path,
    library: &WorldLibrary,
    preferences: &UserPreferences,
) -> Option<PathBuf> {
    let relative = Path::new(preferences.last_world_manifest.as_deref()?);
    if relative.as_os_str().is_empty()
        || relative.is_absolute()
        || !relative
            .components()
            .all(|component| matches!(component, std::path::Component::Normal(_)))
    {
        return None;
    }
    let candidate = project_root.join(relative);
    library
        .entries
        .iter()
        .find(|entry| entry.manifest_path == candidate)
        .map(|entry| entry.manifest_path.clone())
}

fn apply_user_preferences(config: &mut RuntimeConfig, preferences: &UserPreferences) {
    let streaming = &mut config.engine.streaming;
    streaming.load_radius_chunks = preferences.render_distance_chunks;
    streaming.unload_radius_chunks = preferences.render_distance_chunks.saturating_add(1);
    let (near, mid, far): (f32, f32, f32) = match preferences.lod_detail {
        0 => (0.75, 1.5, 2.25),
        1 => (1.0, 1.75, 2.5),
        _ => (1.25, 2.0, 2.75),
    };
    streaming.lod_near_distance_chunks = near;
    streaming.lod_mid_distance_chunks = mid;
    streaming.lod_far_distance_chunks = far.min(streaming.unload_radius_chunks as f32);
}

fn remember_world_manifest(
    project_root: &Path,
    manifest_path: &Path,
    preferences: &mut UserPreferences,
) -> Result<(), String> {
    let relative = manifest_path
        .strip_prefix(project_root)
        .map_err(|error| format!("world manifest is outside the project root: {error}"))?;
    preferences.last_world_manifest = Some(relative.to_string_lossy().replace('\\', "/"));
    save_user_preferences(project_root, preferences)
}

fn lod_detail_label(detail: u8) -> &'static str {
    match detail {
        0 => "PERFORMANCE",
        1 => "BALANCED",
        _ => "QUALITY",
    }
}

impl MenuAsset {
    fn fallback() -> Self {
        let mut fallback: Self = serde_json::from_str(include_str!("../assets/ui/main_menu.json"))
            .expect("the embedded menu theme must be valid JSON");
        fallback.ensure_required_actions();
        fallback
    }

    fn ensure_required_actions(&mut self) {
        if !self.buttons.iter().any(|button| button.action == "resume") {
            self.buttons.push(MenuButtonDefinition {
                label: "RESUME".to_owned(),
                action: "resume".to_owned(),
            });
        }
        if !self.buttons.iter().any(|button| button.action == "quit") {
            self.buttons.push(MenuButtonDefinition {
                label: "QUIT".to_owned(),
                action: "quit".to_owned(),
            });
        }
    }

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
            &self.empty_library_title,
            &self.empty_library_hint,
            &self.quit_confirm_title,
            &self.quit_confirm_message,
            &self.quit_confirm_label,
            &self.quit_cancel_label,
            &self.library_navigation_hint,
            &self.pause_controls_hint,
            &self.pause_title,
            &self.world_count_template,
            &self.library_warning_template,
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
        if !self.world_count_template.contains("{count}")
            || !self.library_warning_template.contains("{count}")
        {
            return Err(
                "world-count and library-warning templates must contain {count}".to_owned(),
            );
        }

        let colors = [
            self.background_srgba,
            self.panel_srgba,
            self.title_srgba,
            self.subtitle_srgba,
            self.button_srgba,
            self.button_hover_srgba,
            self.button_text_srgba,
            self.panel_border_srgba,
            self.panel_shadow_srgba,
            self.button_border_srgba,
            self.button_active_border_srgba,
            self.danger_button_srgba,
            self.danger_button_hover_srgba,
            self.modal_backdrop_srgba,
            self.notice_srgba,
            self.success_srgba,
            self.library_warning_srgba,
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
        if ![
            self.panel_radius_px,
            self.card_radius_px,
            self.panel_shadow_blur_px,
        ]
        .iter()
        .all(|value| value.is_finite() && *value >= 0.0)
            || !self.panel_shadow_offset_y_px.is_finite()
        {
            return Err(
                "panel/card radii and shadow blur must be non-negative; shadow offset must be finite"
                    .to_owned(),
            );
        }
        let home_dimensions = [
            self.home_layout.panel_width_px,
            self.home_layout.panel_height_percent,
            self.home_layout.title_frame_width_px,
            self.home_layout.title_frame_height_px,
            self.home_layout.title_font_size_px,
            self.home_layout.eyebrow_font_size_px,
            self.home_layout.version_font_size_px,
            self.home_layout.actions_top_gap_px,
            self.home_layout.action_list_gap_px,
            self.home_layout.action_row_width_px,
            self.home_layout.action_row_height_px,
            self.home_layout.action_button_width_px,
            self.home_layout.action_button_height_px,
            self.home_layout.action_button_left_px,
            self.home_layout.action_button_padding_left_px,
            self.home_layout.action_font_size_px,
            self.home_layout.cursor_font_size_px,
        ];
        if !home_dimensions
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
            || !self.home_layout.left_percent.is_finite()
            || !(0.0..100.0).contains(&self.home_layout.left_percent)
            || !self.home_layout.top_percent.is_finite()
            || !(0.0..100.0).contains(&self.home_layout.top_percent)
            || self.home_layout.top_percent + self.home_layout.panel_height_percent > 100.0
            || self.home_layout.panel_height_percent > 100.0
            || self.home_layout.title_frame_width_px > self.home_layout.panel_width_px
            || self.home_layout.action_row_width_px > self.home_layout.panel_width_px
            || self.home_layout.action_button_left_px + self.home_layout.action_button_width_px
                > self.home_layout.action_row_width_px
            || self.home_layout.action_button_padding_left_px
                >= self.home_layout.action_button_width_px
            || self.home_layout.action_button_height_px > self.home_layout.action_row_height_px
        {
            return Err(
                "home layout dimensions must be finite, positive, and internally consistent"
                    .to_owned(),
            );
        }

        if self.home_buttons.len() != 4 {
            return Err("menu must define exactly four home buttons".to_owned());
        }
        let expected_home_actions = ["continue", "create_world", "worlds", "settings"];
        let mut home_actions = BTreeSet::new();
        for button in &self.home_buttons {
            if button.label.trim().is_empty()
                || !expected_home_actions.contains(&button.action.as_str())
                || !home_actions.insert(button.action.as_str())
            {
                return Err("home buttons must uniquely define continue, create_world, worlds, and settings with non-empty labels".to_owned());
            }
        }

        if self.buttons.is_empty() {
            return Err("menu must define at least one button".to_owned());
        }

        let mut actions = BTreeSet::new();
        for button in &self.buttons {
            if button.label.trim().is_empty() {
                return Err("menu button labels cannot be empty".to_owned());
            }
            if !matches!(
                button.action.as_str(),
                "start" | "resume" | "quit" | "create_world"
            ) {
                return Err(format!(
                    "unsupported menu action '{}' (expected start, resume, quit, or create_world)",
                    button.action
                ));
            }
            if !actions.insert(button.action.as_str()) {
                return Err(format!("duplicate menu action '{}'", button.action));
            }
        }
        if !actions.contains("resume") || !actions.contains("quit") {
            return Err("menu must define both resume and quit actions".to_owned());
        }
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
fn setup_menu(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut config: ResMut<RuntimeConfig>,
    library: Res<WorldLibrary>,
    mut state: ResMut<MenuState>,
    mut feedback: ResMut<MenuFeedback>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut input_focus: ResMut<InputFocus>,
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
        Ok(menu) => {
            feedback.message.clear();
            feedback.tone = MenuFeedbackTone::Notice;
            menu
        }
        Err(error) => {
            error!(
                "Could not load menu presentation '{}': {error}. Using the built-in theme.",
                menu_path.display()
            );
            feedback.message =
                "The menu theme could not be loaded. The built-in theme is active.".to_owned();
            feedback.tone = MenuFeedbackTone::Notice;
            MenuAsset::fallback()
        }
    };

    let mut preferences = load_user_preferences(&config.paths.project_root);
    // A command-line --world launch is also a joined world. Only remember it when
    // the inferred conventional manifest is present in the discovered library.
    if !config.start_in_library
        && let Some(manifest_path) = config
            .paths
            .world_state
            .parent()
            .map(|directory| directory.join("world.json"))
            .filter(|path| {
                library
                    .entries
                    .iter()
                    .any(|entry| entry.manifest_path == *path)
            })
        && let Err(error) =
            remember_world_manifest(&config.paths.project_root, &manifest_path, &mut preferences)
    {
        warn!(
            "Could not remember command-line world '{}': {error}",
            manifest_path.display()
        );
    }
    apply_user_preferences(&mut config, &preferences);
    let last_world_manifest =
        preferred_world_manifest(&config.paths.project_root, &library, &preferences);
    state.preferences = preferences;
    state.page = MenuPage::Home;

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
    let panel_border_color = color_from_srgba(menu.panel_border_srgba);
    let panel_shadow_color = color_from_srgba(menu.panel_shadow_srgba);
    let modal_backdrop_color = color_from_srgba(menu.modal_backdrop_srgba);
    let notice_color = color_from_srgba(menu.notice_srgba);
    let success_color = color_from_srgba(menu.success_srgba);
    let notice_colors = MenuNoticeColors {
        notice: notice_color,
        success: success_color,
    };
    let library_warning_color = color_from_srgba(menu.library_warning_srgba);
    let button_border_color = color_from_srgba(menu.button_border_srgba);
    let button_active_border_color = color_from_srgba(menu.button_active_border_srgba);
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
                HomeOnly,
                Visibility::Hidden,
                Node {
                    width: Val::Px(menu.home_layout.panel_width_px),
                    height: Val::Percent(menu.home_layout.panel_height_percent),
                    position_type: PositionType::Absolute,
                    left: Val::Percent(menu.home_layout.left_percent),
                    top: Val::Percent(menu.home_layout.top_percent),
                    padding: UiRect::all(Val::Px(0.0)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::FlexStart,
                    justify_content: JustifyContent::FlexStart,
                    row_gap: Val::Px(menu.home_layout.action_list_gap_px),
                    ..default()
                },
                TabGroup::modal(),
            ))
            .with_children(|panel| {
                panel
                    .spawn((
                        Node {
                            width: Val::Px(menu.home_layout.title_frame_width_px),
                            height: Val::Px(menu.home_layout.title_frame_height_px),
                            padding: UiRect::axes(Val::Px(34.0), Val::Px(14.0)),
                            flex_direction: FlexDirection::Column,
                            justify_content: JustifyContent::SpaceBetween,
                            border: UiRect::all(Val::Px(1.0)),
                            border_radius: BorderRadius::all(Val::Px(0.0)),
                            ..default()
                        },
                        BorderColor::all(panel_border_color),
                        BackgroundColor(panel_color),
                        BoxShadow::new(
                            Color::srgba(1.0, 1.0, 1.0, 0.10),
                            Val::Px(0.0),
                            Val::Px(0.0),
                            Val::Px(0.0),
                            Val::Px(18.0),
                        ),
                    ))
                    .with_children(|frame| {
                        frame
                            .spawn((Node {
                                width: Val::Percent(100.0),
                                flex_direction: FlexDirection::Row,
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::SpaceBetween,
                                ..default()
                            },))
                            .with_children(|row| {
                                row.spawn((
                                    Text::new("III"),
                                    TextFont {
                                        font_size: FontSize::Px(
                                            menu.home_layout.eyebrow_font_size_px,
                                        ),
                                        ..default()
                                    },
                                    TextColor(subtitle_color),
                                ));
                                row.spawn((
                                    Text::new("+"),
                                    TextFont {
                                        font_size: FontSize::Px(25.0),
                                        ..default()
                                    },
                                    TextColor(title_color),
                                ));
                            });
                        frame.spawn((
                            Text::new(menu.title.clone()),
                            TextFont {
                                font_size: FontSize::Px(menu.home_layout.title_font_size_px),
                                ..default()
                            },
                            TextColor(title_color),
                        ));
                        frame
                            .spawn((Node {
                                width: Val::Percent(100.0),
                                flex_direction: FlexDirection::Row,
                                justify_content: JustifyContent::FlexEnd,
                                ..default()
                            },))
                            .with_children(|row| {
                                row.spawn((
                                    Text::new(format!("v{}", env!("CARGO_PKG_VERSION"))),
                                    TextFont {
                                        font_size: FontSize::Px(
                                            menu.home_layout.version_font_size_px,
                                        ),
                                        ..default()
                                    },
                                    TextColor(subtitle_color),
                                ));
                            });
                    });

                panel.spawn(Node {
                    width: Val::Px(1.0),
                    height: Val::Px(menu.home_layout.actions_top_gap_px),
                    ..default()
                });

                macro_rules! home_action_button {
                    ($definition:expr, $tab:expr, $action:expr, $observer:expr, $focus:expr) => {{
                        let label = $definition.label.clone();
                        let mut button_entity = None;
                        panel
                            .spawn(Node {
                                width: Val::Px(menu.home_layout.action_row_width_px),
                                height: Val::Px(menu.home_layout.action_row_height_px),
                                position_type: PositionType::Relative,
                                ..default()
                            })
                            .with_children(|item| {
                                let button = item
                                    .spawn((
                                        Button,
                                        TabIndex($tab),
                                        Hovered::default(),
                                        Node {
                                            width: Val::Px(menu.home_layout.action_button_width_px),
                                            height: Val::Px(
                                                menu.home_layout.action_button_height_px,
                                            ),
                                            margin: UiRect::left(Val::Px(
                                                menu.home_layout.action_button_left_px,
                                            )),
                                            padding: UiRect::left(Val::Px(
                                                menu.home_layout.action_button_padding_left_px,
                                            )),
                                            border: UiRect::all(Val::Px(1.5)),
                                            border_radius: BorderRadius::all(Val::Px(0.0)),
                                            align_items: AlignItems::Center,
                                            justify_content: JustifyContent::FlexStart,
                                            ..default()
                                        },
                                        BorderColor::all(button_border_color),
                                        BackgroundColor(button_color),
                                        MenuAction($action.to_owned()),
                                        MenuButtonColors {
                                            normal: button_color,
                                            hovered: button_hover_color,
                                            border_normal: button_border_color,
                                            border_active: button_active_border_color,
                                        },
                                        MenuGlowState::default(),
                                        BoxShadow::new(
                                            Color::srgba(1.0, 1.0, 1.0, 0.0),
                                            Val::Px(0.0),
                                            Val::Px(0.0),
                                            Val::Px(0.0),
                                            Val::Px(16.0),
                                        ),
                                        observe($observer),
                                    ))
                                    .with_children(|button| {
                                        button.spawn((
                                            Text::new(label),
                                            TextFont {
                                                font_size: FontSize::Px(
                                                    menu.home_layout.action_font_size_px,
                                                ),
                                                ..default()
                                            },
                                            TextColor(button_text_color),
                                        ));
                                    })
                                    .id();
                                button_entity = Some(button);
                                item.spawn((
                                    MenuCursorMarker(button),
                                    Visibility::Hidden,
                                    Node {
                                        width: Val::Px(28.0),
                                        height: Val::Px(menu.home_layout.cursor_font_size_px + 9.0),
                                        position_type: PositionType::Absolute,
                                        left: Val::Px(0.0),
                                        top: Val::Px(
                                            (menu.home_layout.action_row_height_px
                                                - menu.home_layout.cursor_font_size_px)
                                                * 0.5,
                                        ),
                                        ..default()
                                    },
                                    Text::new("▶"),
                                    TextFont {
                                        font_size: FontSize::Px(
                                            menu.home_layout.cursor_font_size_px,
                                        ),
                                        ..default()
                                    },
                                    TextColor(title_color),
                                ));
                                item.spawn((
                                    Node {
                                        width: Val::Px(18.0),
                                        height: Val::Px(18.0),
                                        position_type: PositionType::Absolute,
                                        left: Val::Px(490.0),
                                        top: Val::Px(-2.0),
                                        ..default()
                                    },
                                    Text::new("╱"),
                                    TextFont {
                                        font_size: FontSize::Px(20.0),
                                        ..default()
                                    },
                                    TextColor(title_color),
                                ));
                                item.spawn((
                                    Node {
                                        width: Val::Px(18.0),
                                        height: Val::Px(18.0),
                                        position_type: PositionType::Absolute,
                                        left: Val::Px(32.0),
                                        top: Val::Px(58.0),
                                        ..default()
                                    },
                                    Text::new("╱"),
                                    TextFont {
                                        font_size: FontSize::Px(20.0),
                                        ..default()
                                    },
                                    TextColor(title_color),
                                ));
                            });
                        if $focus {
                            input_focus.set(
                                button_entity.expect("home action button was created"),
                                FocusCause::Navigated,
                            );
                        }
                    }};
                }

                let continue_definition = menu
                    .home_buttons
                    .iter()
                    .find(|definition| definition.action == "continue")
                    .expect("validated home menu contains Continue");
                if let Some(last_world_manifest) = last_world_manifest.clone() {
                    let label = continue_definition.label.clone();
                    let mut button_entity = None;
                    panel
                        .spawn(Node {
                            width: Val::Px(menu.home_layout.action_row_width_px),
                            height: Val::Px(menu.home_layout.action_row_height_px),
                            position_type: PositionType::Relative,
                            ..default()
                        })
                        .with_children(|item| {
                            let button = item
                                .spawn((
                                    Button,
                                    TabIndex(0),
                                    Hovered::default(),
                                    Node {
                                        width: Val::Px(menu.home_layout.action_button_width_px),
                                        height: Val::Px(menu.home_layout.action_button_height_px),
                                        margin: UiRect::left(Val::Px(
                                            menu.home_layout.action_button_left_px,
                                        )),
                                        padding: UiRect::left(Val::Px(
                                            menu.home_layout.action_button_padding_left_px,
                                        )),
                                        border: UiRect::all(Val::Px(1.5)),
                                        border_radius: BorderRadius::all(Val::Px(0.0)),
                                        align_items: AlignItems::Center,
                                        justify_content: JustifyContent::FlexStart,
                                        ..default()
                                    },
                                    BorderColor::all(button_border_color),
                                    BackgroundColor(button_color),
                                    WorldSelection(last_world_manifest),
                                    MenuButtonColors {
                                        normal: button_color,
                                        hovered: button_hover_color,
                                        border_normal: button_border_color,
                                        border_active: button_active_border_color,
                                    },
                                    MenuGlowState::default(),
                                    BoxShadow::new(
                                        Color::srgba(1.0, 1.0, 1.0, 0.0),
                                        Val::Px(0.0),
                                        Val::Px(0.0),
                                        Val::Px(0.0),
                                        Val::Px(16.0),
                                    ),
                                    observe(handle_world_selection),
                                ))
                                .with_children(|button| {
                                    button.spawn((
                                        Text::new(label),
                                        TextFont {
                                            font_size: FontSize::Px(
                                                menu.home_layout.action_font_size_px,
                                            ),
                                            ..default()
                                        },
                                        TextColor(button_text_color),
                                    ));
                                })
                                .id();
                            button_entity = Some(button);
                            item.spawn((
                                MenuCursorMarker(button),
                                Visibility::Hidden,
                                Node {
                                    width: Val::Px(28.0),
                                    height: Val::Px(menu.home_layout.cursor_font_size_px + 9.0),
                                    position_type: PositionType::Absolute,
                                    left: Val::Px(0.0),
                                    top: Val::Px(
                                        (menu.home_layout.action_row_height_px
                                            - menu.home_layout.cursor_font_size_px)
                                            * 0.5,
                                    ),
                                    ..default()
                                },
                                Text::new("▶"),
                                TextFont {
                                    font_size: FontSize::Px(menu.home_layout.cursor_font_size_px),
                                    ..default()
                                },
                                TextColor(title_color),
                            ));
                            item.spawn((
                                Node {
                                    width: Val::Px(18.0),
                                    height: Val::Px(18.0),
                                    position_type: PositionType::Absolute,
                                    left: Val::Px(490.0),
                                    top: Val::Px(-2.0),
                                    ..default()
                                },
                                Text::new("╱"),
                                TextFont {
                                    font_size: FontSize::Px(20.0),
                                    ..default()
                                },
                                TextColor(title_color),
                            ));
                            item.spawn((
                                Node {
                                    width: Val::Px(18.0),
                                    height: Val::Px(18.0),
                                    position_type: PositionType::Absolute,
                                    left: Val::Px(32.0),
                                    top: Val::Px(58.0),
                                    ..default()
                                },
                                Text::new("╱"),
                                TextFont {
                                    font_size: FontSize::Px(20.0),
                                    ..default()
                                },
                                TextColor(title_color),
                            ));
                        });
                    input_focus.set(
                        button_entity.expect("Continue button was created"),
                        FocusCause::Navigated,
                    );
                } else {
                    panel
                        .spawn(Node {
                            width: Val::Px(menu.home_layout.action_row_width_px),
                            height: Val::Px(menu.home_layout.action_row_height_px),
                            position_type: PositionType::Relative,
                            ..default()
                        })
                        .with_children(|item| {
                            item.spawn((
                                Node {
                                    width: Val::Px(menu.home_layout.action_button_width_px),
                                    height: Val::Px(menu.home_layout.action_button_height_px),
                                    margin: UiRect::left(Val::Px(
                                        menu.home_layout.action_button_left_px,
                                    )),
                                    padding: UiRect::left(Val::Px(
                                        menu.home_layout.action_button_padding_left_px,
                                    )),
                                    border: UiRect::all(Val::Px(1.0)),
                                    border_radius: BorderRadius::all(Val::Px(0.0)),
                                    align_items: AlignItems::Center,
                                    justify_content: JustifyContent::FlexStart,
                                    ..default()
                                },
                                BorderColor::all(panel_border_color),
                                BackgroundColor(panel_color),
                            ))
                            .with_children(|button| {
                                button.spawn((
                                    Text::new("Continue  /  NO RECENT WORLD"),
                                    TextFont {
                                        font_size: FontSize::Px(17.0),
                                        ..default()
                                    },
                                    TextColor(subtitle_color),
                                ));
                            });
                        });
                }

                let new_world = menu
                    .home_buttons
                    .iter()
                    .find(|definition| definition.action == "create_world")
                    .expect("validated home menu contains New World");
                home_action_button!(
                    new_world,
                    1,
                    "create_world",
                    handle_create_world,
                    last_world_manifest.is_none()
                );

                let worlds = menu
                    .home_buttons
                    .iter()
                    .find(|definition| definition.action == "worlds")
                    .expect("validated home menu contains Worlds");
                home_action_button!(worlds, 2, "worlds", handle_navigation_action, false);

                let settings = menu
                    .home_buttons
                    .iter()
                    .find(|definition| definition.action == "settings")
                    .expect("validated home menu contains Settings");
                home_action_button!(settings, 3, "settings", handle_navigation_action, false);

                panel.spawn((
                    MenuNotice,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(menu.world_description_size_px),
                        ..default()
                    },
                    TextColor(notice_color),
                    notice_colors,
                    Node {
                        width: Val::Px(menu.home_layout.action_row_width_px),
                        min_height: Val::Px(18.0),
                        margin: UiRect::left(Val::Px(menu.home_layout.action_button_left_px)),
                        ..default()
                    },
                ));

                panel.spawn((
                    Text::new("TAB / SHIFT+TAB  ·  ENTER SELECT"),
                    TextFont {
                        font_size: FontSize::Px(menu.world_metadata_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));
            });

            root.spawn((
                SettingsOnly,
                Visibility::Hidden,
                Node {
                    width: Val::Percent(92.0),
                    max_width: Val::Px(650.0),
                    padding: UiRect::all(Val::Px(menu.panel_padding_px + 12.0)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Stretch,
                    row_gap: Val::Px(menu.vertical_gap_px),
                    border: UiRect::all(Val::Px(1.0)),
                    border_radius: BorderRadius::all(Val::Px(0.0)),
                    ..default()
                },
                BorderColor::all(panel_border_color),
                BackgroundColor(panel_color),
                BoxShadow::new(
                    panel_shadow_color,
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_offset_y_px),
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_blur_px),
                ),
                TabGroup::modal(),
            ))
            .with_children(|panel| {
                panel.spawn((
                    Text::new("SETTINGS"),
                    TextFont {
                        font_size: FontSize::Px(menu.title_size_px + 14.0),
                        ..default()
                    },
                    TextColor(title_color),
                ));
                panel.spawn((
                    Text::new("STREAMING / VISUAL QUALITY"),
                    TextFont {
                        font_size: FontSize::Px(menu.world_metadata_size_px + 2.0),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                panel
                    .spawn(Node {
                        width: Val::Percent(100.0),
                        height: Val::Px(1.0),
                        margin: UiRect::vertical(Val::Px(12.0)),
                        ..default()
                    })
                    .insert(BackgroundColor(panel_border_color));

                panel
                    .spawn(Node {
                        width: Val::Percent(100.0),
                        min_height: Val::Px(64.0),
                        flex_direction: FlexDirection::Row,
                        align_items: AlignItems::Center,
                        column_gap: Val::Px(12.0),
                        ..default()
                    })
                    .with_children(|row| {
                        row.spawn((
                            Text::new("RENDER DISTANCE"),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                            Node {
                                flex_grow: 1.0,
                                ..default()
                            },
                        ));
                        row.spawn((
                            SettingsRenderDistanceValue,
                            Text::new(""),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                            Node {
                                width: Val::Px(104.0),
                                ..default()
                            },
                        ));
                        for (label, action, index) in [
                            ("−", "render_distance_down", 0),
                            ("+", "render_distance_up", 1),
                        ] {
                            row.spawn((
                                Button,
                                TabIndex(4 + index),
                                Hovered::default(),
                                Node {
                                    width: Val::Px(42.0),
                                    height: Val::Px(42.0),
                                    border: UiRect::all(Val::Px(1.0)),
                                    align_items: AlignItems::Center,
                                    justify_content: JustifyContent::Center,
                                    ..default()
                                },
                                BorderColor::all(button_border_color),
                                BackgroundColor(button_color),
                                MenuButtonColors {
                                    normal: button_color,
                                    hovered: button_hover_color,
                                    border_normal: button_border_color,
                                    border_active: button_active_border_color,
                                },
                                SettingAction(action.to_owned()),
                                observe(handle_setting_action),
                            ))
                            .with_children(|button| {
                                button.spawn((
                                    Text::new(label),
                                    TextFont {
                                        font_size: FontSize::Px(menu.button_font_size_px + 4.0),
                                        ..default()
                                    },
                                    TextColor(button_text_color),
                                ));
                            });
                        }
                    });

                panel
                    .spawn(Node {
                        width: Val::Percent(100.0),
                        min_height: Val::Px(64.0),
                        flex_direction: FlexDirection::Row,
                        align_items: AlignItems::Center,
                        column_gap: Val::Px(12.0),
                        ..default()
                    })
                    .with_children(|row| {
                        row.spawn((
                            Text::new("LOD DETAIL"),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                            Node {
                                flex_grow: 1.0,
                                ..default()
                            },
                        ));
                        row.spawn((
                            SettingsLodDetailValue,
                            Text::new(""),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                            Node {
                                width: Val::Px(130.0),
                                ..default()
                            },
                        ));
                        for (label, action, index) in
                            [("−", "lod_detail_down", 0), ("+", "lod_detail_up", 1)]
                        {
                            row.spawn((
                                Button,
                                TabIndex(6 + index),
                                Hovered::default(),
                                Node {
                                    width: Val::Px(42.0),
                                    height: Val::Px(42.0),
                                    border: UiRect::all(Val::Px(1.0)),
                                    align_items: AlignItems::Center,
                                    justify_content: JustifyContent::Center,
                                    ..default()
                                },
                                BorderColor::all(button_border_color),
                                BackgroundColor(button_color),
                                MenuButtonColors {
                                    normal: button_color,
                                    hovered: button_hover_color,
                                    border_normal: button_border_color,
                                    border_active: button_active_border_color,
                                },
                                SettingAction(action.to_owned()),
                                observe(handle_setting_action),
                            ))
                            .with_children(|button| {
                                button.spawn((
                                    Text::new(label),
                                    TextFont {
                                        font_size: FontSize::Px(menu.button_font_size_px + 4.0),
                                        ..default()
                                    },
                                    TextColor(button_text_color),
                                ));
                            });
                        }
                    });

                panel.spawn((
                    Text::new("Changes apply live and are saved automatically."),
                    TextFont {
                        font_size: FontSize::Px(menu.world_metadata_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));
                panel.spawn((
                    MenuNotice,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(menu.world_description_size_px),
                        ..default()
                    },
                    TextColor(notice_color),
                    notice_colors,
                    Node {
                        width: Val::Percent(100.0),
                        min_height: Val::Px(18.0),
                        ..default()
                    },
                ));

                panel
                    .spawn((
                        Button,
                        TabIndex(8),
                        Hovered::default(),
                        Node {
                            width: Val::Percent(100.0),
                            height: Val::Px(menu.button_height_px),
                            margin: UiRect::top(Val::Px(10.0)),
                            border: UiRect::all(Val::Px(1.0)),
                            align_items: AlignItems::Center,
                            justify_content: JustifyContent::Center,
                            ..default()
                        },
                        BorderColor::all(button_border_color),
                        BackgroundColor(button_color),
                        MenuAction("home".to_owned()),
                        MenuButtonColors {
                            normal: button_color,
                            hovered: button_hover_color,
                            border_normal: button_border_color,
                            border_active: button_active_border_color,
                        },
                        observe(handle_navigation_action),
                    ))
                    .with_children(|button| {
                        button.spawn((
                            Text::new("BACK"),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                        ));
                    });
            });

            root.spawn((
                LibraryOnly,
                Visibility::Hidden,
                Node {
                    width: Val::Percent(92.0),
                    max_width: Val::Px(menu.panel_width_px),
                    height: Val::Percent(88.0),
                    max_height: Val::Percent(88.0),
                    min_height: Val::Px(0.0),
                    padding: UiRect::all(Val::Px(menu.panel_padding_px)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Stretch,
                    row_gap: Val::Px(menu.vertical_gap_px),
                    overflow: Overflow::hidden(),
                    border: UiRect::all(Val::Px(1.0)),
                    border_radius: BorderRadius::all(Val::Px(menu.panel_radius_px)),
                    ..default()
                },
                BorderColor::all(panel_border_color),
                BackgroundColor(panel_color),
                BoxShadow::new(
                    panel_shadow_color,
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_offset_y_px),
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_blur_px),
                ),
                TabGroup::modal(),
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
                panel.spawn((
                    Text::new(
                        menu.world_count_template
                            .replace("{count}", &library.entries.len().to_string()),
                    ),
                    TextFont {
                        font_size: FontSize::Px(menu.world_metadata_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                if !library.warnings.is_empty() {
                    panel.spawn((
                        Text::new(
                            menu.library_warning_template
                                .replace("{count}", &library.warnings.len().to_string()),
                        ),
                        TextFont {
                            font_size: FontSize::Px(menu.world_metadata_size_px),
                            ..default()
                        },
                        TextColor(library_warning_color),
                    ));
                }

                panel
                    .spawn((
                        WorldList,
                        Node {
                            width: Val::Percent(100.0),
                            min_height: Val::Px(0.0),
                            flex_grow: 1.0,
                            flex_direction: FlexDirection::Column,
                            row_gap: Val::Px(menu.world_card_gap_px),
                            overflow: Overflow::scroll_y(),
                            scrollbar_width: 7.0,
                            ..default()
                        },
                        ScrollPosition::default(),
                    ))
                    .with_children(|world_list| {
                        if library.entries.is_empty() {
                            world_list
                                .spawn((
                                    Node {
                                        width: Val::Percent(100.0),
                                        padding: UiRect::all(Val::Px(24.0)),
                                        flex_direction: FlexDirection::Column,
                                        align_items: AlignItems::FlexStart,
                                        row_gap: Val::Px(10.0),
                                        border: UiRect::all(Val::Px(1.0)),
                                        border_radius: BorderRadius::all(Val::Px(
                                            menu.card_radius_px,
                                        )),
                                        ..default()
                                    },
                                    BorderColor::all(panel_border_color),
                                    BackgroundColor(button_color),
                                ))
                                .with_children(|empty| {
                                    empty.spawn((
                                        Text::new(menu.empty_library_title.clone()),
                                        TextFont {
                                            font_size: FontSize::Px(menu.button_font_size_px),
                                            ..default()
                                        },
                                        TextColor(button_text_color),
                                    ));
                                    empty.spawn((
                                        Text::new(menu.empty_library_hint.clone()),
                                        TextFont {
                                            font_size: FontSize::Px(menu.world_description_size_px),
                                            ..default()
                                        },
                                        TextColor(subtitle_color),
                                    ));
                                });
                        }

                        for (world_index, world) in library.entries.iter().enumerate() {
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
                            let metadata_lines = world_metadata_lines(
                                &world.metadata,
                                menu.world_metadata_max_fields,
                            );
                            let format_summary = format!(
                                "{} v{}  |  {} {}  |  {} {}",
                                menu.world_format_label,
                                world.schema_version,
                                world.region_count,
                                menu.world_regions_label,
                                world.object_count,
                                menu.world_objects_label
                            );

                            world_list
                                .spawn((
                                    Button,
                                    TabIndex(world_index as i32),
                                    Hovered::default(),
                                    Node {
                                        width: Val::Percent(100.0),
                                        min_height: Val::Px(menu.world_card_height_px),
                                        border: UiRect::all(Val::Px(1.0)),
                                        border_radius: BorderRadius::all(Val::Px(
                                            menu.card_radius_px,
                                        )),
                                        padding: UiRect::all(Val::Px(12.0)),
                                        flex_direction: FlexDirection::Row,
                                        column_gap: Val::Px(menu.world_card_gap_px),
                                        align_items: AlignItems::Center,
                                        justify_content: JustifyContent::FlexStart,
                                        ..default()
                                    },
                                    BorderColor::all(button_border_color),
                                    BackgroundColor(button_color),
                                    WorldSelection(world.manifest_path.clone()),
                                    MenuButtonColors {
                                        normal: button_color,
                                        hovered: button_hover_color,
                                        border_normal: button_border_color,
                                        border_active: button_active_border_color,
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
                                                border_radius: BorderRadius::all(Val::Px(
                                                    menu.card_radius_px,
                                                )),
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
                                                    border_radius: BorderRadius::all(Val::Px(
                                                        menu.card_radius_px,
                                                    )),
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
                                                    font_size: FontSize::Px(
                                                        menu.button_font_size_px,
                                                    ),
                                                    ..default()
                                                },
                                                TextColor(button_text_color),
                                            ));
                                            details.spawn((
                                                Text::new(description.clone()),
                                                TextFont {
                                                    font_size: FontSize::Px(
                                                        menu.world_description_size_px,
                                                    ),
                                                    ..default()
                                                },
                                                TextColor(subtitle_color),
                                            ));
                                            details.spawn((
                                                Text::new(format_summary.clone()),
                                                TextFont {
                                                    font_size: FontSize::Px(
                                                        menu.world_metadata_size_px,
                                                    ),
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
                                    button.spawn((
                                        Text::new("JOIN  ▸"),
                                        TextFont {
                                            font_size: FontSize::Px(
                                                menu.world_metadata_size_px + 1.0,
                                            ),
                                            ..default()
                                        },
                                        TextColor(title_color),
                                    ));
                                });
                        }
                    });

                panel
                    .spawn((
                        Button,
                        TabIndex(library.entries.len() as i32),
                        Hovered::default(),
                        Node {
                            width: Val::Percent(100.0),
                            max_width: Val::Px(menu.button_width_px),
                            align_self: AlignSelf::Center,
                            height: Val::Px(menu.button_height_px),
                            border: UiRect::all(Val::Px(1.0)),
                            border_radius: BorderRadius::all(Val::Px(menu.card_radius_px)),
                            align_items: AlignItems::Center,
                            justify_content: JustifyContent::Center,
                            ..default()
                        },
                        BorderColor::all(button_border_color),
                        BackgroundColor(button_color),
                        MenuAction("home".to_owned()),
                        MenuButtonColors {
                            normal: button_color,
                            hovered: button_hover_color,
                            border_normal: button_border_color,
                            border_active: button_active_border_color,
                        },
                        observe(handle_navigation_action),
                    ))
                    .with_children(|button| {
                        button.spawn((
                            Text::new("BACK TO MENU"),
                            TextFont {
                                font_size: FontSize::Px(menu.button_font_size_px),
                                ..default()
                            },
                            TextColor(button_text_color),
                        ));
                    });

                panel.spawn((
                    Text::new(menu.library_navigation_hint.clone()),
                    TextFont {
                        font_size: FontSize::Px(menu.world_metadata_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                panel.spawn((
                    MenuNotice,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(menu.world_description_size_px),
                        ..default()
                    },
                    TextColor(notice_color),
                    notice_colors,
                    Node {
                        width: Val::Percent(100.0),
                        min_height: Val::Px(18.0),
                        ..default()
                    },
                ));

                if let Some(create_world) = menu
                    .buttons
                    .iter()
                    .find(|button| button.action == "create_world")
                {
                    panel
                        .spawn((
                            Button,
                            TabIndex(library.entries.len() as i32 + 1),
                            Hovered::default(),
                            Node {
                                width: Val::Percent(100.0),
                                max_width: Val::Px(menu.button_width_px),
                                align_self: AlignSelf::Center,
                                height: Val::Px(menu.button_height_px),
                                border: UiRect::all(Val::Px(1.0)),
                                border_radius: BorderRadius::all(Val::Px(menu.card_radius_px)),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::Center,
                                ..default()
                            },
                            BorderColor::all(button_border_color),
                            BackgroundColor(button_color),
                            MenuAction(create_world.action.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                                border_normal: button_border_color,
                                border_active: button_active_border_color,
                            },
                            observe(handle_create_world),
                        ))
                        .with_children(|button| {
                            button.spawn((
                                Text::new(create_world.label.clone()),
                                TextFont {
                                    font_size: FontSize::Px(menu.button_font_size_px),
                                    ..default()
                                },
                                TextColor(button_text_color),
                            ));
                        });
                }

                if let Some(quit) = menu.buttons.iter().find(|button| button.action == "quit") {
                    panel
                        .spawn((
                            Button,
                            TabIndex(
                                library.entries.len() as i32
                                    + 1
                                    + if menu
                                        .buttons
                                        .iter()
                                        .any(|button| button.action == "create_world")
                                    {
                                        1
                                    } else {
                                        0
                                    },
                            ),
                            Hovered::default(),
                            Node {
                                width: Val::Percent(100.0),
                                max_width: Val::Px(menu.button_width_px),
                                align_self: AlignSelf::Center,
                                height: Val::Px(menu.button_height_px),
                                border: UiRect::all(Val::Px(1.0)),
                                border_radius: BorderRadius::all(Val::Px(menu.card_radius_px)),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::Center,
                                ..default()
                            },
                            BorderColor::all(button_border_color),
                            BackgroundColor(button_color),
                            MenuAction(quit.action.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                                border_normal: button_border_color,
                                border_active: button_active_border_color,
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
                TabGroup::modal(),
                Visibility::Hidden,
                Node {
                    width: Val::Percent(92.0),
                    max_width: Val::Px(menu.panel_width_px),
                    padding: UiRect::all(Val::Px(menu.panel_padding_px)),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Stretch,
                    row_gap: Val::Px(menu.vertical_gap_px),
                    border: UiRect::all(Val::Px(1.0)),
                    border_radius: BorderRadius::all(Val::Px(menu.panel_radius_px)),
                    ..default()
                },
                BorderColor::all(panel_border_color),
                BackgroundColor(panel_color),
                BoxShadow::new(
                    panel_shadow_color,
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_offset_y_px),
                    Val::Px(0.0),
                    Val::Px(menu.panel_shadow_blur_px),
                ),
            ))
            .with_children(|panel| {
                panel.spawn((
                    Text::new(menu.pause_title.clone()),
                    TextFont {
                        font_size: FontSize::Px(menu.title_size_px),
                        ..default()
                    },
                    TextColor(title_color),
                ));
                panel.spawn((
                    Text::new(menu.pause_controls_hint.clone()),
                    TextFont {
                        font_size: FontSize::Px(menu.subtitle_size_px),
                        ..default()
                    },
                    TextColor(subtitle_color),
                ));

                panel.spawn((
                    MenuNotice,
                    Text::new(""),
                    TextFont {
                        font_size: FontSize::Px(menu.world_description_size_px),
                        ..default()
                    },
                    TextColor(notice_color),
                    notice_colors,
                    Node {
                        width: Val::Percent(100.0),
                        min_height: Val::Px(18.0),
                        ..default()
                    },
                ));

                for (index, definition) in menu
                    .buttons
                    .iter()
                    .filter(|button| matches!(button.action.as_str(), "resume" | "quit"))
                    .enumerate()
                {
                    panel
                        .spawn((
                            Button,
                            TabIndex(index as i32),
                            Hovered::default(),
                            Node {
                                width: Val::Percent(100.0),
                                max_width: Val::Px(menu.button_width_px),
                                align_self: AlignSelf::Center,
                                height: Val::Px(menu.button_height_px),
                                border: UiRect::all(Val::Px(1.0)),
                                border_radius: BorderRadius::all(Val::Px(menu.card_radius_px)),
                                align_items: AlignItems::Center,
                                justify_content: JustifyContent::Center,
                                ..default()
                            },
                            BorderColor::all(button_border_color),
                            BackgroundColor(button_color),
                            MenuAction(definition.action.clone()),
                            MenuButtonColors {
                                normal: button_color,
                                hovered: button_hover_color,
                                border_normal: button_border_color,
                                border_active: button_active_border_color,
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

            root.spawn((
                QuitConfirmationPanel,
                Visibility::Hidden,
                Node {
                    position_type: PositionType::Absolute,
                    left: Val::Px(0.0),
                    right: Val::Px(0.0),
                    top: Val::Px(0.0),
                    bottom: Val::Px(0.0),
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    align_items: AlignItems::Center,
                    justify_content: JustifyContent::Center,
                    padding: UiRect::all(Val::Px(20.0)),
                    ..default()
                },
                BackgroundColor(modal_backdrop_color),
                TabGroup::modal(),
            ))
            .with_children(|overlay| {
                overlay
                    .spawn((
                        Node {
                            width: Val::Percent(92.0),
                            max_width: Val::Px(560.0),
                            padding: UiRect::all(Val::Px(menu.panel_padding_px)),
                            flex_direction: FlexDirection::Column,
                            align_items: AlignItems::Stretch,
                            row_gap: Val::Px(menu.vertical_gap_px),
                            border: UiRect::all(Val::Px(1.0)),
                            border_radius: BorderRadius::all(Val::Px(menu.panel_radius_px)),
                            ..default()
                        },
                        BorderColor::all(panel_border_color),
                        BackgroundColor(panel_color),
                        BoxShadow::new(
                            panel_shadow_color,
                            Val::Px(0.0),
                            Val::Px(menu.panel_shadow_offset_y_px),
                            Val::Px(0.0),
                            Val::Px(menu.panel_shadow_blur_px),
                        ),
                    ))
                    .with_children(|dialog| {
                        dialog.spawn((
                            Text::new(menu.quit_confirm_title.clone()),
                            TextFont {
                                font_size: FontSize::Px(menu.title_size_px),
                                ..default()
                            },
                            TextColor(title_color),
                        ));
                        dialog.spawn((
                            Text::new(menu.quit_confirm_message.clone()),
                            TextFont {
                                font_size: FontSize::Px(menu.subtitle_size_px),
                                ..default()
                            },
                            TextColor(subtitle_color),
                        ));

                        dialog.spawn((
                            MenuNotice,
                            Text::new(""),
                            TextFont {
                                font_size: FontSize::Px(menu.world_description_size_px),
                                ..default()
                            },
                            TextColor(notice_color),
                            notice_colors,
                            Node {
                                width: Val::Percent(100.0),
                                min_height: Val::Px(18.0),
                                ..default()
                            },
                        ));

                        let danger_button_color = color_from_srgba(menu.danger_button_srgba);
                        let danger_button_hover_color =
                            color_from_srgba(menu.danger_button_hover_srgba);

                        for (index, (label, action)) in [
                            (menu.quit_cancel_label.clone(), "quit_cancel"),
                            (menu.quit_confirm_label.clone(), "quit_confirm"),
                        ]
                        .into_iter()
                        .enumerate()
                        {
                            let (normal_color, hovered_color) = if action == "quit_confirm" {
                                (danger_button_color, danger_button_hover_color)
                            } else {
                                (button_color, button_hover_color)
                            };

                            dialog
                                .spawn((
                                    Button,
                                    TabIndex(index as i32),
                                    Hovered::default(),
                                    Node {
                                        width: Val::Percent(100.0),
                                        min_height: Val::Px(menu.button_height_px),
                                        border: UiRect::all(Val::Px(1.0)),
                                        border_radius: BorderRadius::all(Val::Px(
                                            menu.card_radius_px,
                                        )),
                                        align_items: AlignItems::Center,
                                        justify_content: JustifyContent::Center,
                                        ..default()
                                    },
                                    BorderColor::all(button_border_color),
                                    BackgroundColor(normal_color),
                                    MenuAction(action.to_owned()),
                                    MenuButtonColors {
                                        normal: normal_color,
                                        hovered: hovered_color,
                                        border_normal: button_border_color,
                                        border_active: button_active_border_color,
                                    },
                                    observe(handle_menu_action),
                                ))
                                .with_children(|button| {
                                    button.spawn((
                                        Text::new(label),
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
    mut world_origin: ResMut<WorldOrigin>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut menu_state: ResMut<MenuState>,
    mut feedback: ResMut<MenuFeedback>,
    mut input_focus: ResMut<InputFocus>,
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
            feedback.message = format!(
                "Could not open '{}'. Check the application log for details.",
                selection.0.display()
            );
            feedback.tone = MenuFeedbackTone::Notice;
            return;
        }
    };
    selected.paths.project_root = project_root.clone();
    apply_user_preferences(&mut selected, &menu_state.preferences);

    let selected_state =
        match WorldStateFile::load_or_default(&selected.paths.world_state, &selected.world.id) {
            Ok(state) => state,
            Err(error) => {
                error!(
                    "Could not load save state for world '{}': {error:#}",
                    selected.world.id
                );
                feedback.message = format!(
                    "Could not load the save for '{}'. Check the application log for details.",
                    selected.world.name
                );
                feedback.tone = MenuFeedbackTone::Notice;
                return;
            }
        };

    if let Err(error) =
        remember_world_manifest(&project_root, &selection.0, &mut menu_state.preferences)
    {
        warn!(
            "Could not remember selected world '{}': {error}",
            selection.0.display()
        );
    }

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
    world_origin.0 = DVec3::from_array(selected_state.origin_m);
    *world_state = selected_state;
    *config = selected;
    feedback.message.clear();
    input_focus.clear();

    menu_state.visible = false;
    virtual_time.unpause();
    next_state.set(AppMode::Simulation);
    info!(
        "Selected world '{}'. Starting simulation.",
        config.world.name
    );
}

fn write_flat_platform_world(project_root: &Path) -> Result<PathBuf, String> {
    let worlds_directory = project_root.join("worlds");
    fs::create_dir_all(&worlds_directory)
        .map_err(|error| format!("could not create world library directory: {error}"))?;

    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let world_id = format!("flat-platform-{timestamp}");
    let world_directory = worlds_directory.join(&world_id);
    fs::create_dir(&world_directory)
        .map_err(|error| format!("could not reserve a unique world directory: {error}"))?;

    let manifest = serde_json::json!({
        "schema_version": 1,
        "id": world_id,
        "name": "Flat Platform (255,000,000 units)",
        "description": "Procedurally streamed flat bedrock platform with a raised perimeter.",
        "preview_asset": null,
        "metadata": {
            "generation": "flat_platform",
            "platform_size_units": 255000000_u64,
            "border_height_units": 4,
            "border_thickness_units": 2,
            "chunk_edge_units": 32
        },
        "gravity_m_s2": null,
        "generation": {
            "kind": "flat_platform",
            "size_units": 255000000_u64,
            "border_height_units": 4,
            "border_thickness_units": 2
        },
        "regions": [],
        "objects": []
    });
    let contents = serde_json::to_vec_pretty(&manifest)
        .map_err(|error| format!("could not serialize generated world manifest: {error}"))?;
    let manifest_path = world_directory.join("world.json");
    let temporary_path = world_directory.join(".world.json.tmp");
    fs::write(&temporary_path, contents)
        .map_err(|error| format!("could not write generated world manifest: {error}"))?;
    if let Err(error) = fs::rename(&temporary_path, &manifest_path) {
        let _ = fs::remove_file(&temporary_path);
        let _ = fs::remove_dir(&world_directory);
        return Err(format!(
            "could not commit generated world manifest: {error}"
        ));
    }
    Ok(manifest_path)
}

/// Creates a data-only world recipe and starts it without materializing the full platform.
#[allow(clippy::too_many_arguments)]
fn handle_create_world(
    activate: On<Activate>,
    actions: Query<&MenuAction>,
    mode: Res<State<AppMode>>,
    mut config: ResMut<RuntimeConfig>,
    mut world_state: ResMut<WorldStateFile>,
    mut save_target: ResMut<WorldSaveTarget>,
    mut journal: ResMut<DestructionJournal>,
    mut gravity: ResMut<Gravity>,
    mut world_origin: ResMut<WorldOrigin>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut menu_state: ResMut<MenuState>,
    mut feedback: ResMut<MenuFeedback>,
    mut input_focus: ResMut<InputFocus>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if *mode.get() != AppMode::WorldLibrary
        || !actions
            .get(activate.entity)
            .is_ok_and(|action| action.0 == "create_world")
    {
        return;
    }

    let project_root = config.paths.project_root.clone();
    let manifest_path = match write_flat_platform_world(&project_root) {
        Ok(path) => path,
        Err(error) => {
            error!("Could not create flat platform world: {error}");
            feedback.message = format!("World creation failed: {error}");
            feedback.tone = MenuFeedbackTone::Notice;
            return;
        }
    };

    let mut selected = match RuntimeConfig::load(
        config.paths.data_dir.clone(),
        config.paths.assets_dir.clone(),
        manifest_path.clone(),
    ) {
        Ok(config) => config,
        Err(error) => {
            error!(
                "Generated world '{}' failed validation: {error:#}",
                manifest_path.display()
            );
            feedback.message =
                "Generated world failed validation. Check the application log.".to_owned();
            feedback.tone = MenuFeedbackTone::Notice;
            return;
        }
    };
    selected.paths.project_root = project_root.clone();
    apply_user_preferences(&mut selected, &menu_state.preferences);

    let selected_state =
        match WorldStateFile::load_or_default(&selected.paths.world_state, &selected.world.id) {
            Ok(state) => state,
            Err(error) => {
                error!("Could not initialize save state for generated world: {error:#}");
                feedback.message = "Could not initialize the new world's save state.".to_owned();
                feedback.tone = MenuFeedbackTone::Notice;
                return;
            }
        };

    if let Err(error) =
        remember_world_manifest(&project_root, &manifest_path, &mut menu_state.preferences)
    {
        warn!(
            "Could not remember new world '{}': {error}",
            manifest_path.display()
        );
    }

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
    world_origin.0 = DVec3::from_array(selected_state.origin_m);
    *world_state = selected_state;
    *config = selected;

    feedback.message.clear();
    input_focus.clear();
    menu_state.visible = false;
    virtual_time.unpause();
    next_state.set(AppMode::Simulation);
    info!(
        "Created and started procedural world '{}'.",
        config.world.id
    );
}

fn load_menu(path: &Path) -> Result<MenuAsset, String> {
    let source =
        std::fs::read_to_string(path).map_err(|error| format!("failed to read file: {error}"))?;
    let mut menu: MenuAsset =
        serde_json::from_str(&source).map_err(|error| format!("invalid menu JSON: {error}"))?;
    menu.ensure_required_actions();
    menu.validate()?;
    Ok(menu)
}

fn handle_navigation_action(
    activate: On<Activate>,
    actions: Query<&MenuAction>,
    mode: Res<State<AppMode>>,
    mut state: ResMut<MenuState>,
    mut input_focus: ResMut<InputFocus>,
) {
    if *mode.get() != AppMode::WorldLibrary {
        return;
    }
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };
    state.page = match action.0.as_str() {
        "home" => MenuPage::Home,
        "worlds" => MenuPage::Worlds,
        "settings" => MenuPage::Settings,
        _ => return,
    };
    input_focus.clear();
}

fn handle_setting_action(
    activate: On<Activate>,
    actions: Query<&SettingAction>,
    mode: Res<State<AppMode>>,
    mut config: ResMut<RuntimeConfig>,
    mut state: ResMut<MenuState>,
    mut feedback: ResMut<MenuFeedback>,
) {
    if *mode.get() != AppMode::WorldLibrary || state.page != MenuPage::Settings {
        return;
    }
    let Ok(action) = actions.get(activate.entity) else {
        return;
    };

    match action.0.as_str() {
        "render_distance_down" => {
            state.preferences.render_distance_chunks = state
                .preferences
                .render_distance_chunks
                .saturating_sub(1)
                .max(2);
        }
        "render_distance_up" => {
            state.preferences.render_distance_chunks =
                (state.preferences.render_distance_chunks + 1).min(8);
        }
        "lod_detail_down" => {
            state.preferences.lod_detail = state.preferences.lod_detail.saturating_sub(1);
        }
        "lod_detail_up" => {
            state.preferences.lod_detail = (state.preferences.lod_detail + 1).min(2);
        }
        _ => return,
    }

    apply_user_preferences(&mut config, &state.preferences);
    match save_user_preferences(&config.paths.project_root, &state.preferences) {
        Ok(()) => {
            feedback.message = format!(
                "Settings saved · render distance {} chunks · LOD {}",
                state.preferences.render_distance_chunks,
                lod_detail_label(state.preferences.lod_detail)
            );
            feedback.tone = MenuFeedbackTone::Success;
        }
        Err(error) => {
            warn!("Could not save settings: {error}");
            feedback.message =
                format!("Settings applied for this session, but could not be saved: {error}");
            feedback.tone = MenuFeedbackTone::Notice;
        }
    }
}

fn sync_settings_labels(
    config: Res<RuntimeConfig>,
    state: Res<MenuState>,
    mut render_distance_labels: Query<
        &mut Text,
        (
            With<SettingsRenderDistanceValue>,
            Without<SettingsLodDetailValue>,
        ),
    >,
    mut lod_detail_labels: Query<
        &mut Text,
        (
            With<SettingsLodDetailValue>,
            Without<SettingsRenderDistanceValue>,
        ),
    >,
) {
    if !config.is_changed() && !state.is_changed() {
        return;
    }

    for mut text in &mut render_distance_labels {
        text.0 = format!("{} CHUNKS", config.engine.streaming.load_radius_chunks);
    }
    for mut text in &mut lod_detail_labels {
        text.0 = lod_detail_label(state.preferences.lod_detail).to_owned();
    }
}

// Bevy requires these independent resources as explicit system parameters. Keeping them
// separate preserves the scheduler's access information and avoids a generic catch-all state.
#[allow(clippy::too_many_arguments)]
fn handle_menu_action(
    activate: On<Activate>,
    actions: Query<(Entity, &MenuAction)>,
    mode: Res<State<AppMode>>,
    mut state: ResMut<MenuState>,
    mut confirmation: ResMut<QuitConfirmationState>,
    mut input_focus: ResMut<InputFocus>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok((_, action)) = actions.get(activate.entity) else {
        return;
    };

    match action.0.as_str() {
        "start" | "resume" => {
            state.visible = false;
            input_focus.clear();
            virtual_time.unpause();
        }
        "quit" if *mode.get() == AppMode::Simulation => {
            confirmation.visible = true;
            confirmation.return_focus = Some(activate.entity);
            if let Some((cancel, _)) = actions.iter().find(|(_, action)| action.0 == "quit_cancel")
            {
                input_focus.set(cancel, FocusCause::Navigated);
            }
        }
        "quit" => {
            app_exit.write(AppExit::Success);
        }
        "quit_cancel" => {
            confirmation.visible = false;
            if let Some(previous) = confirmation.return_focus.take() {
                input_focus.set(previous, FocusCause::Navigated);
            } else {
                input_focus.clear();
            }
        }
        "quit_confirm" => {
            app_exit.write(AppExit::Success);
        }
        _ => unreachable!("menu actions are validated during asset loading"),
    }
}

fn sync_quit_confirmation_visibility(
    confirmation: Res<QuitConfirmationState>,
    mut panels: Query<&mut Visibility, With<QuitConfirmationPanel>>,
) {
    for mut visibility in &mut panels {
        visibility.set_if_neq(if confirmation.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
}

fn sync_menu_feedback(
    feedback: Res<MenuFeedback>,
    mut notices: Query<(&mut Text, &mut TextColor, &MenuNoticeColors), With<MenuNotice>>,
) {
    if !feedback.is_changed() {
        return;
    }

    for (mut text, mut text_color, colors) in &mut notices {
        text.0.clone_from(&feedback.message);
        text_color.0 = match feedback.tone {
            MenuFeedbackTone::Notice => colors.notice,
            MenuFeedbackTone::Success => colors.success,
        };
    }
}

fn scroll_position_after_delta(
    current: f32,
    delta: f32,
    visible_height: f32,
    content_height: f32,
    inverse_scale_factor: f32,
) -> f32 {
    let max_offset = (content_height - visible_height).max(0.0) * inverse_scale_factor;
    (current - delta).clamp(0.0, max_offset)
}

fn update_world_list_scroll(
    mut mouse_wheel_reader: MessageReader<MouseWheel>,
    hover_map: Res<HoverMap>,
    mut scrollable: Query<(&mut ScrollPosition, &ComputedNode), With<WorldList>>,
) {
    for wheel in mouse_wheel_reader.read() {
        let dy = match wheel.unit {
            MouseScrollUnit::Line => wheel.y * 24.0,
            MouseScrollUnit::Pixel => wheel.y,
        };

        for pointer_map in hover_map.values() {
            for entity in pointer_map.keys() {
                let Ok((mut position, computed)) = scrollable.get_mut(*entity) else {
                    continue;
                };
                position.0.y = scroll_position_after_delta(
                    position.0.y,
                    dy,
                    computed.size().y,
                    computed.content_size().y,
                    computed.inverse_scale_factor,
                );
            }
        }
    }
}

fn toggle_menu_with_escape(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<AppMode>>,
    mut state: ResMut<MenuState>,
    actions: Query<(Entity, &MenuAction)>,
    mut confirmation: ResMut<QuitConfirmationState>,
    mut input_focus: ResMut<InputFocus>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if !keys.just_pressed(KeyCode::Escape) {
        return;
    }

    if confirmation.visible {
        confirmation.visible = false;
        if let Some(previous) = confirmation.return_focus.take() {
            input_focus.set(previous, FocusCause::Navigated);
        } else {
            input_focus.clear();
        }
        return;
    }

    if *mode.get() == AppMode::WorldLibrary {
        if state.page != MenuPage::Home {
            state.page = MenuPage::Home;
            input_focus.clear();
        }
        return;
    }

    state.visible = !state.visible;
    if state.visible {
        virtual_time.pause();
        if let Some((resume, _)) = actions.iter().find(|(_, action)| action.0 == "resume") {
            input_focus.set(resume, FocusCause::Navigated);
        }
    } else {
        input_focus.clear();
        virtual_time.unpause();
    }
}

type MenuButtonStyleQuery<'world, 'state> = Query<
    'world,
    'state,
    (
        Entity,
        &'static Hovered,
        Has<Pressed>,
        &'static MenuButtonColors,
        &'static mut BackgroundColor,
        &'static mut BorderColor,
        Option<&'static mut MenuGlowState>,
        Option<&'static mut BoxShadow>,
    ),
    With<Button>,
>;

fn update_button_styles(
    input_focus: Res<InputFocus>,
    mut buttons: MenuButtonStyleQuery<'_, '_>,
    mut cursor_markers: Query<(&MenuCursorMarker, &mut Visibility)>,
) {
    let mut active_buttons = Vec::new();
    let focused_entity = input_focus.get();

    for (entity, hovered, pressed, colors, mut background, mut border, glow_state, shadow) in
        &mut buttons
    {
        let focused = focused_entity == Some(entity);
        let active = pressed || hovered.get() || focused;
        let cursor_active = pressed || hovered.get() || focused;
        if cursor_active {
            active_buttons.push(entity);
        }

        background.0 = if active {
            colors.hovered
        } else {
            colors.normal
        };
        *border = BorderColor::all(if active {
            colors.border_active
        } else {
            colors.border_normal
        });

        if let (Some(mut glow_state), Some(mut shadow)) = (glow_state, shadow)
            && glow_state.0 != active
        {
            *shadow = BoxShadow::new(
                Color::srgba(1.0, 1.0, 1.0, if active { 0.20 } else { 0.0 }),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(0.0),
                Val::Px(16.0),
            );
            glow_state.0 = active;
        }
    }

    for (marker, mut visibility) in &mut cursor_markers {
        visibility.set_if_neq(if active_buttons.contains(&marker.0) {
            Visibility::Visible
        } else {
            Visibility::Hidden
        });
    }
}

type RootVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<MenuRoot>,
        Without<HomeOnly>,
        Without<LibraryOnly>,
        Without<SettingsOnly>,
        Without<SimulationOnly>,
    ),
>;
type HomeVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<HomeOnly>,
        Without<MenuRoot>,
        Without<LibraryOnly>,
        Without<SettingsOnly>,
        Without<SimulationOnly>,
    ),
>;
type LibraryVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<LibraryOnly>,
        Without<MenuRoot>,
        Without<HomeOnly>,
        Without<SettingsOnly>,
        Without<SimulationOnly>,
    ),
>;
type SettingsVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<SettingsOnly>,
        Without<MenuRoot>,
        Without<HomeOnly>,
        Without<LibraryOnly>,
        Without<SimulationOnly>,
    ),
>;
type SimulationVisibilityQuery<'world, 'state> = Query<
    'world,
    'state,
    &'static mut Visibility,
    (
        With<SimulationOnly>,
        Without<MenuRoot>,
        Without<HomeOnly>,
        Without<LibraryOnly>,
        Without<SettingsOnly>,
    ),
>;

fn sync_menu_visibility(
    state: Res<MenuState>,
    mode: Res<State<AppMode>>,
    mut roots: RootVisibilityQuery<'_, '_>,
    mut home_panels: HomeVisibilityQuery<'_, '_>,
    mut library_panels: LibraryVisibilityQuery<'_, '_>,
    mut settings_panels: SettingsVisibilityQuery<'_, '_>,
    mut simulation_panels: SimulationVisibilityQuery<'_, '_>,
) {
    let in_library = *mode.get() == AppMode::WorldLibrary;
    let (root_visibility, _, simulation_visibility) =
        menu_visibility_targets(in_library, state.visible);
    let home_visibility = if in_library && state.page == MenuPage::Home {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let library_visibility = if in_library && state.page == MenuPage::Worlds {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    let settings_visibility = if in_library && state.page == MenuPage::Settings {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
    if state.is_changed() || mode.is_changed() {
        info!(
            "Menu state changed: mode={:?}, page={:?}, menu_visible={}, root={:?}, home={:?}, worlds={:?}, settings={:?}, pause={:?}",
            mode.get(),
            state.page,
            state.visible,
            root_visibility,
            home_visibility,
            library_visibility,
            settings_visibility,
            simulation_visibility
        );
    }

    for mut visibility in &mut roots {
        visibility.set_if_neq(root_visibility);
    }
    for mut visibility in &mut home_panels {
        visibility.set_if_neq(home_visibility);
    }
    for mut visibility in &mut library_panels {
        visibility.set_if_neq(library_visibility);
    }
    for mut visibility in &mut settings_panels {
        visibility.set_if_neq(settings_visibility);
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
    use super::*;
    use bevy::prelude::Visibility;
    use std::collections::BTreeMap;

    fn valid_menu() -> MenuAsset {
        MenuAsset {
            schema_version: 1,
            title: "Title".to_owned(),
            subtitle: "Subtitle".to_owned(),
            home_buttons: default_home_buttons(),
            home_layout: HomeLayoutDefinition::default(),
            empty_library_title: default_empty_library_title(),
            empty_library_hint: default_empty_library_hint(),
            quit_confirm_title: default_quit_confirm_title(),
            quit_confirm_message: default_quit_confirm_message(),
            quit_confirm_label: default_quit_confirm_label(),
            quit_cancel_label: default_quit_cancel_label(),
            library_navigation_hint: default_library_navigation_hint(),
            pause_controls_hint: default_pause_controls_hint(),
            pause_title: default_pause_title(),
            world_count_template: default_world_count_template(),
            library_warning_template: default_library_warning_template(),
            modal_backdrop_srgba: default_modal_backdrop_srgba(),
            notice_srgba: default_notice_srgba(),
            success_srgba: default_success_srgba(),
            library_warning_srgba: default_library_warning_srgba(),
            background_srgba: [0.0, 0.0, 0.0, 1.0],
            panel_srgba: [0.1, 0.1, 0.1, 1.0],
            title_srgba: [1.0; 4],
            subtitle_srgba: [1.0; 4],
            button_srgba: [0.1, 0.1, 0.1, 1.0],
            button_hover_srgba: [0.2, 0.2, 0.2, 1.0],
            button_text_srgba: [1.0; 4],
            panel_border_srgba: default_panel_border_srgba(),
            panel_shadow_srgba: default_panel_shadow_srgba(),
            panel_shadow_blur_px: default_panel_shadow_blur_px(),
            panel_shadow_offset_y_px: default_panel_shadow_offset_y_px(),
            button_border_srgba: default_button_border_srgba(),
            button_active_border_srgba: default_button_active_border_srgba(),
            danger_button_srgba: default_danger_button_srgba(),
            danger_button_hover_srgba: default_danger_button_hover_srgba(),
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
            panel_radius_px: default_panel_radius_px(),
            card_radius_px: default_card_radius_px(),
            buttons: vec![
                MenuButtonDefinition {
                    label: "Resume".to_owned(),
                    action: "resume".to_owned(),
                },
                MenuButtonDefinition {
                    label: "Quit".to_owned(),
                    action: "quit".to_owned(),
                },
            ],
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

        let mut menu: MenuAsset = serde_json::from_value(legacy).unwrap();
        menu.ensure_required_actions();
        assert!(menu.validate().is_ok());
        assert!(menu.buttons.iter().any(|button| button.action == "resume"));
        assert_eq!(menu.world_card_height_px, 116.0);
        assert_eq!(menu.world_preview_fallback_label, "WORLD");
        assert_eq!(menu.world_metadata_max_fields, 4);
        assert_eq!(menu.empty_library_title, "YOUR WORLD LIBRARY IS EMPTY");
        assert_eq!(menu.panel_radius_px, 18.0);
        assert_eq!(menu.card_radius_px, 12.0);
        assert_eq!(menu.panel_shadow_blur_px, 30.0);
        assert_eq!(menu.panel_shadow_offset_y_px, 12.0);
        assert_eq!(menu.quit_confirm_title, "UNSAVED SIMULATION");
        assert_eq!(menu.quit_confirm_label, "QUIT ANYWAY");
        assert_eq!(menu.quit_cancel_label, "KEEP PLAYING");
        assert_eq!(menu.danger_button_srgba, [0.31, 0.075, 0.095, 1.0]);
        assert_eq!(
            menu.library_navigation_hint,
            default_library_navigation_hint()
        );
        assert_eq!(menu.pause_title, "SIMULATION PAUSED");
        assert_eq!(menu.world_count_template, "{count} WORLDS AVAILABLE");
        assert_eq!(menu.modal_backdrop_srgba, default_modal_backdrop_srgba());
        assert_eq!(menu.success_srgba, [0.28, 0.78, 0.65, 1.0]);
        assert_eq!(menu.quit_confirm_title, "UNSAVED SIMULATION");
        assert_eq!(menu.quit_cancel_label, "KEEP PLAYING");
        assert_eq!(menu.danger_button_srgba, [0.31, 0.075, 0.095, 1.0]);
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
    fn scroll_position_clamps_to_content_and_respects_ui_scale() {
        let next = scroll_position_after_delta(150.0, 80.0, 300.0, 1100.0, 0.5);
        assert_eq!(next, 70.0);

        let at_start = scroll_position_after_delta(0.0, 50.0, 300.0, 1100.0, 0.5);
        assert_eq!(at_start, 0.0);

        let at_end = scroll_position_after_delta(390.0, -50.0, 300.0, 1100.0, 0.5);
        assert_eq!(at_end, 400.0);
    }

    #[test]
    fn fallback_theme_is_valid_and_retains_essential_actions() {
        let fallback = MenuAsset::fallback();
        assert!(fallback.validate().is_ok());
        assert!(
            fallback
                .buttons
                .iter()
                .any(|button| button.action == "resume")
        );
        assert!(
            fallback
                .buttons
                .iter()
                .any(|button| button.action == "quit")
        );
    }

    #[test]
    fn menu_asset_rejects_invalid_home_layout_geometry() {
        let mut menu = valid_menu();
        menu.home_layout.left_percent = f32::NAN;
        assert!(menu.validate().is_err());

        let mut menu = valid_menu();
        menu.home_layout.action_button_width_px = menu.home_layout.action_row_width_px;
        assert!(menu.validate().is_err());
    }

    #[test]
    fn menu_theme_allows_flat_panels_and_disabled_shadows() {
        let mut menu = valid_menu();
        menu.panel_radius_px = 0.0;
        menu.card_radius_px = 0.0;
        menu.panel_shadow_blur_px = 0.0;
        menu.panel_shadow_offset_y_px = 0.0;
        assert!(menu.validate().is_ok());
    }

    #[test]
    fn menu_asset_requires_unique_resume_and_quit_actions() {
        let mut menu = valid_menu();
        menu.buttons.pop();
        assert!(menu.validate().is_err());

        let mut menu = valid_menu();
        menu.buttons.push(MenuButtonDefinition {
            label: "Another quit".to_owned(),
            action: "quit".to_owned(),
        });
        assert!(menu.validate().is_err());
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

    fn temp_root(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        std::env::temp_dir().join(format!("25124-ui-{label}-{nonce}"))
    }

    #[test]
    fn user_preferences_round_trip_and_reject_out_of_range_values() {
        let root = temp_root("preferences");
        fs::create_dir_all(&root).unwrap();

        let mut preferences = UserPreferences {
            last_world_manifest: Some("worlds/alpha/world.json".to_owned()),
            render_distance_chunks: 8,
            lod_detail: 2,
            ..UserPreferences::default()
        };
        save_user_preferences(&root, &preferences).unwrap();
        assert_eq!(load_user_preferences(&root), preferences);

        preferences.render_distance_chunks = 9;
        save_user_preferences(&root, &preferences).unwrap();
        assert_eq!(load_user_preferences(&root), UserPreferences::default());

        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn continue_target_must_be_relative_and_present_in_the_world_library() {
        let root = temp_root("recent-world");
        let world_directory = root.join("worlds/alpha");
        fs::create_dir_all(&world_directory).unwrap();
        fs::write(
            world_directory.join("world.json"),
            r#"{"id":"alpha","name":"Alpha"}"#,
        )
        .unwrap();

        let library = WorldLibrary::discover(&root).unwrap();
        let mut preferences = UserPreferences {
            last_world_manifest: Some("worlds/alpha/world.json".to_owned()),
            ..UserPreferences::default()
        };
        assert_eq!(
            preferred_world_manifest(&root, &library, &preferences),
            Some(world_directory.join("world.json"))
        );

        for unsafe_or_stale in ["../outside/world.json", "worlds/missing/world.json", ""] {
            preferences.last_world_manifest = Some(unsafe_or_stale.to_owned());
            assert_eq!(
                preferred_world_manifest(&root, &library, &preferences),
                None
            );
        }

        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn lod_detail_profiles_preserve_strict_distance_ordering() {
        let profiles = [
            (0.75_f32, 1.5_f32, 2.25_f32),
            (1.0_f32, 1.75_f32, 2.5_f32),
            (1.25_f32, 2.0_f32, 2.75_f32),
        ];
        for (near, mid, far) in profiles {
            assert!(near > 0.0 && near < mid && mid < far && far <= 3.0);
        }
    }
}
