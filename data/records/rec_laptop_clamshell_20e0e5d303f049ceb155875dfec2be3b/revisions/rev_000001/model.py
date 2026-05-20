from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BASE_LENGTH = 0.312
BASE_WIDTH = 0.220
BASE_HALF_LENGTH = BASE_LENGTH * 0.5
BASE_HALF_WIDTH = BASE_WIDTH * 0.5
KEYBOARD_PLANE_Z = 0.0102
HINGE_X = 0.146
HINGE_Z = 0.0159


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + (height * 0.5))
        for y_pos, z_pos in rounded_rect_profile(width, height, radius)
    ]


def _rotate_y(point: tuple[float, float, float], pitch: float) -> tuple[float, float, float]:
    x_pos, y_pos, z_pos = point
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    return (
        (x_pos * cos_pitch) + (z_pos * sin_pitch),
        y_pos,
        (-x_pos * sin_pitch) + (z_pos * cos_pitch),
    )


def _pitched_origin(
    local_center: tuple[float, float, float],
    pitch: float,
) -> Origin:
    return Origin(xyz=_rotate_y(local_center, pitch), rpy=(0.0, pitch, 0.0))


def _build_base_shell():
    return section_loft(
        [
            _yz_section(
                -BASE_HALF_LENGTH,
                width=0.202,
                height=0.0064,
                radius=0.0028,
            ),
            _yz_section(
                -0.112,
                width=0.214,
                height=0.0104,
                radius=0.0046,
            ),
            _yz_section(
                0.018,
                width=0.220,
                height=0.0132,
                radius=0.0056,
            ),
            _yz_section(
                BASE_HALF_LENGTH,
                width=0.220,
                height=0.0164,
                radius=0.0066,
            ),
        ]
    )


def _add_key(
    model: ArticulatedObject,
    base,
    *,
    name: str,
    x_pos: float,
    y_pos: float,
    size: tuple[float, float, float],
    mass: float,
):
    key = model.part(name)
    key.visual(
        Box(size),
        origin=Origin(xyz=(0.0, 0.0, 0.0012)),
        material="keycap",
        name="cap",
    )
    key.visual(
        Box((size[0] * 0.42, size[1] * 0.38, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, -0.00060)),
        material="key_shadow",
        name="stem",
    )
    key.inertial = Inertial.from_geometry(
        Box((size[0], size[1], 0.0030)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
    )

    model.articulation(
        f"base_to_{name}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key,
        origin=Origin(xyz=(x_pos, y_pos, KEYBOARD_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.05,
            lower=-0.00040,
            upper=0.0,
        ),
    )
    return key


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thin_laptop")

    model.material("aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("anodized_dark", rgba=(0.22, 0.24, 0.28, 1.0))
    model.material("hinge_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("screen_glass", rgba=(0.07, 0.09, 0.10, 0.92))
    model.material("keyboard_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("trackpad_glass", rgba=(0.36, 0.38, 0.41, 1.0))
    model.material("keycap", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("key_shadow", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("accent_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_shell(), "laptop_base_shell"),
        material="aluminum",
        name="outer_shell",
    )
    base.visual(
        Box((0.246, 0.184, 0.0010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0097)),
        material="keyboard_black",
        name="keyboard_plate",
    )
    base.visual(
        Box((0.110, 0.070, 0.0006)),
        origin=Origin(xyz=(-0.063, 0.0, 0.01005)),
        material="trackpad_glass",
        name="trackpad",
    )
    base.visual(
        Box((0.112, 0.004, 0.0026)),
        origin=Origin(xyz=(-0.149, 0.0, 0.0030)),
        material="accent_dark",
        name="front_jaw_edge",
    )
    base.visual(
        Box((0.190, 0.014, 0.0020)),
        origin=Origin(xyz=(0.090, 0.0, 0.0136)),
        material="accent_dark",
        name="rear_hinge_bridge",
    )
    base.visual(
        Cylinder(radius=0.0032, length=0.034),
        origin=Origin(
            xyz=(HINGE_X, -0.067, HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="hinge_dark",
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.0032, length=0.034),
        origin=Origin(
            xyz=(HINGE_X, 0.067, HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="hinge_dark",
        name="right_hinge_barrel",
    )
    base.visual(
        Box((0.034, 0.0018, 0.0060)),
        origin=Origin(xyz=(0.098, BASE_HALF_WIDTH - 0.0009, 0.0079)),
        material="accent_dark",
        name="port_recess",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.020)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    display = model.part("display")
    display_pitch = 0.33
    lid_thickness = 0.0055
    lid_width = 0.286
    lid_height = 0.190
    display.visual(
        Box((lid_thickness, lid_width, lid_height)),
        origin=_pitched_origin((-lid_thickness * 0.5, 0.0, lid_height * 0.5), display_pitch),
        material="anodized_dark",
        name="lid_shell",
    )
    display.visual(
        Box((0.0010, 0.258, 0.160)),
        origin=_pitched_origin((-0.00410, 0.0, 0.103), display_pitch),
        material="screen_glass",
        name="screen_glass",
    )
    display.visual(
        Box((0.0012, 0.250, 0.012)),
        origin=_pitched_origin((-0.0039, 0.0, 0.0082), display_pitch),
        material="keyboard_black",
        name="lower_bezel",
    )
    display.visual(
        Box((0.0010, 0.010, 0.0035)),
        origin=_pitched_origin((-0.0038, 0.0, 0.183), display_pitch),
        material="accent_dark",
        name="webcam_bar",
    )
    display.visual(
        Cylinder(radius=0.0030, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_dark",
        name="center_knuckle",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.060, lid_width, lid_height)),
        mass=0.82,
        origin=Origin(xyz=(0.026, 0.0, 0.092)),
    )
    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.92,
            upper=0.28,
        ),
    )

    port_door = model.part("port_door")
    door_length = 0.038
    door_thickness = 0.0014
    door_height = 0.010
    port_door.visual(
        Box((door_length, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_length * 0.5, door_thickness * 0.5, door_height * 0.5)
        ),
        material="anodized_dark",
        name="door_panel",
    )
    port_door.visual(
        Cylinder(radius=0.0013, length=door_height),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, door_height * 0.5)),
        material="hinge_dark",
        name="door_hinge",
    )
    port_door.visual(
        Box((0.007, 0.0008, 0.0025)),
        origin=Origin(xyz=(door_length - 0.004, door_thickness + 0.0004, door_height * 0.5)),
        material="accent_dark",
        name="door_pull",
    )
    port_door.inertial = Inertial.from_geometry(
        Box((door_length, 0.004, door_height)),
        mass=0.012,
        origin=Origin(xyz=(door_length * 0.5, 0.002, door_height * 0.5)),
    )
    model.articulation(
        "base_to_port_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=port_door,
        origin=Origin(xyz=(0.080, BASE_HALF_WIDTH + 0.00045, 0.0046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    _add_key(
        model,
        base,
        name="key_q",
        x_pos=0.032,
        y_pos=-0.050,
        size=(0.029, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="key_w",
        x_pos=0.032,
        y_pos=0.000,
        size=(0.029, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="key_e",
        x_pos=0.032,
        y_pos=0.050,
        size=(0.029, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="key_a",
        x_pos=0.000,
        y_pos=-0.050,
        size=(0.031, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="key_s",
        x_pos=0.000,
        y_pos=0.000,
        size=(0.031, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="key_d",
        x_pos=0.000,
        y_pos=0.050,
        size=(0.031, 0.024, 0.0014),
        mass=0.004,
    )
    _add_key(
        model,
        base,
        name="spacebar",
        x_pos=-0.044,
        y_pos=0.000,
        size=(0.094, 0.024, 0.0014),
        mass=0.008,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    display = object_model.get_part("display")
    port_door = object_model.get_part("port_door")
    key_q = object_model.get_part("key_q")
    key_w = object_model.get_part("key_w")
    key_e = object_model.get_part("key_e")
    key_a = object_model.get_part("key_a")
    key_s = object_model.get_part("key_s")
    key_d = object_model.get_part("key_d")
    spacebar = object_model.get_part("spacebar")

    display_hinge = object_model.get_articulation("base_to_display")
    door_hinge = object_model.get_articulation("base_to_port_door")
    key_s_slide = object_model.get_articulation("base_to_key_s")
    spacebar_slide = object_model.get_articulation("base_to_spacebar")

    ctx.check(
        "main articulated parts exist",
        all(
            part is not None
            for part in (
                base,
                display,
                port_door,
                key_q,
                key_w,
                key_e,
                key_a,
                key_s,
                key_d,
                spacebar,
            )
        ),
    )

    for key_part in (key_q, key_w, key_e, key_a, key_s, key_d, spacebar):
        ctx.allow_overlap(
            key_part,
            base,
            elem_a="stem",
            reason="The key stem is intentionally represented as plunging through a simplified solid keyboard deck proxy.",
        )

    ctx.expect_gap(
        port_door,
        base,
        axis="y",
        min_gap=0.00005,
        max_gap=0.0015,
        positive_elem="door_panel",
        negative_elem="outer_shell",
        name="port door sits nearly flush with right side wall",
    )

    rest_door_aabb = ctx.part_world_aabb(port_door)
    with ctx.pose({door_hinge: 1.0}):
        open_door_aabb = ctx.part_world_aabb(port_door)
    ctx.check(
        "port door swings outward",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.020,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({display_hinge: -1.78}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            min_gap=0.0001,
            max_gap=0.0015,
            positive_elem="lid_shell",
            negative_elem="keyboard_plate",
            name="display inner face closes close above the keyboard deck",
        )
        ctx.expect_overlap(
            display,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="keyboard_plate",
            min_overlap=0.150,
            name="display covers the keyboard deck footprint when folded",
        )

    ctx.expect_gap(
        key_s,
        base,
        axis="z",
        min_gap=0.00035,
        max_gap=0.0014,
        positive_elem="cap",
        negative_elem="keyboard_plate",
        name="key sits slightly proud of keyboard deck",
    )

    rest_key_aabb = ctx.part_world_aabb(key_s)
    with ctx.pose({key_s_slide: -0.00035, spacebar_slide: -0.00025}):
        ctx.expect_gap(
            key_s,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.0010,
            max_penetration=0.0,
            positive_elem="cap",
            negative_elem="keyboard_plate",
            name="key can depress without clipping through the deck",
        )
        pressed_key_aabb = ctx.part_world_aabb(key_s)
    ctx.check(
        "key plunger motion is downward",
        rest_key_aabb is not None
        and pressed_key_aabb is not None
        and pressed_key_aabb[0][2] < rest_key_aabb[0][2] - 0.0002,
        details=f"rest={rest_key_aabb}, pressed={pressed_key_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
