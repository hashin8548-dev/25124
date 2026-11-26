use std::path::Path;

use bevy::{
    app::AppExit,
    prelude::*,
    text::{FontSize, TextFont},
    ui::widget::Button,
};
use serde::Deserialize;

use crate::config::RuntimeConfig;

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MenuState>()
            .add_systems(Startup, setup_menu)
            .add_systems(
                Update,
                (
                    handle_menu_buttons,
                    toggle_menu_with_escape,
                    update_button_styles,
                    sync_menu_visibility,
                ),
            );
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
    buttons: Vec<MenuButtonDefinition>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MenuButtonDefinition {
    label: String,
    action: String,
}

impl MenuAsset {
    fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 {
            return Err(format!(
                "unsupported menu schema version {}",
                self.schema_version
            ));
        }
        if self.title.trim().is_empty() || self.subtitle.trim().is_empty() {
            return Err("menu title and subtitle cannot be empty".to_owned());
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
        ];
        if !dimensions.iter().all(|value| value.is_finite() && *value > 0.0) {
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
    config: Res<RuntimeConfig>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    let menu_path = config.paths.assets_dir.join("ui/main_menu.json");
    let menu = match load_menu(&menu_path) {
        Ok(menu) => menu,
        Err(error) => {
            error!(
                "Could not load external main menu '{}': {error}. Starting simulation without a menu.",
                menu_path.display()
            );
            state.visible = false;
            virtual_time.unpause();
            return;
        }
    };

    state.visible = true;
    virtual_time.pause();

    let background_color = color_from_srgba(menu.background_srgba);
    let panel_color = color_from_srgba(menu.panel_srgba);
    let title_color = color_from_srgba(menu.title_srgba);
    let subtitle_color = color_from_srgba(menu.subtitle_srgba);
    let button_color = color_from_srgba(menu.button_srgba);
    let button_hover_color = color_from_srgba(menu.button_hover_srgba);
    let button_text_color = color_from_srgba(menu.button_text_srgba);

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
            Visibility::Visible,
        ))
        .with_children(|root| {
            root.spawn((
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

                for definition in &menu.buttons {
                    panel
                        .spawn((
                            Button,
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

fn load_menu(path: &Path) -> Result<MenuAsset, String> {
    let source = std::fs::read_to_string(path)
        .map_err(|error| format!("failed to read file: {error}"))?;
    let menu: MenuAsset = serde_json::from_str(&source)
        .map_err(|error| format!("invalid menu JSON: {error}"))?;
    menu.validate()?;
    Ok(menu)
}

fn handle_menu_buttons(
    buttons: Query<(&Interaction, &MenuAction), (Changed<Interaction>, With<Button>)>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
    mut app_exit: MessageWriter<AppExit>,
) {
    for (interaction, action) in &buttons {
        if *interaction != Interaction::Pressed {
            continue;
        }

        match action.0.as_str() {
            "start" | "resume" => {
                state.visible = false;
                virtual_time.unpause();
            }
            "quit" => app_exit.write(AppExit::Success),
            _ => unreachable!("menu actions are validated during asset loading"),
        }
    }
}

fn toggle_menu_with_escape(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<MenuState>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if !keys.just_pressed(KeyCode::Escape) {
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
        (&Interaction, &MenuButtonColors, &mut BackgroundColor),
        (Changed<Interaction>, With<Button>),
    >,
) {
    for (interaction, colors, mut background) in &mut buttons {
        background.0 = match interaction {
            Interaction::Hovered | Interaction::Pressed => colors.hovered,
            Interaction::None => colors.normal,
        };
    }
}

fn sync_menu_visibility(
    state: Res<MenuState>,
    mut roots: Query<&mut Visibility, With<MenuRoot>>,
) {
    if !state.is_changed() {
        return;
    }

    for mut visibility in &mut roots {
        *visibility = if state.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

fn color_from_srgba(value: [f32; 4]) -> Color {
    Color::srgba(value[0], value[1], value[2], value[3])
}

#[cfg(test)]
mod tests {
    use super::{MenuAsset, MenuButtonDefinition};

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
            buttons: vec![MenuButtonDefinition {
                label: "Start".to_owned(),
                action: "start".to_owned(),
            }],
        }
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
