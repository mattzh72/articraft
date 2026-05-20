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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _panel_point(
    center: tuple[float, float, float],
    roll: float,
    local_x: float,
    local_y: float,
    local_z: float = 0.0,
) -> tuple[float, float, float]:
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cx, cy, cz = center
    return (
        cx + local_x,
        cy + local_y * cos_r - local_z * sin_r,
        cz + local_y * sin_r + local_z * cos_r,
    )


def _ring_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
    outer = [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)]
    inner = [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _circle_profile(radius: float, *, segments: int = 32):
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _button_deck_mesh():
    deck_width = 0.620
    deck_depth = 0.180
    deck_thickness = 0.036
    button_hole_center = (0.200, -0.010)
    button_hole_radius = 0.017
    outer_profile = [
        (-deck_width * 0.5, -deck_depth * 0.5),
        (deck_width * 0.5, -deck_depth * 0.5),
        (deck_width * 0.5, deck_depth * 0.5),
        (-deck_width * 0.5, deck_depth * 0.5),
    ]
    hole_profile = [
        (button_hole_center[0] + x, button_hole_center[1] + y)
        for x, y in _circle_profile(button_hole_radius, segments=40)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [hole_profile],
            deck_thickness,
            cap=True,
            center=True,
        ),
        "casino_button_deck",
    )


def _cabinet_shell_mesh(width: float):
    side_profile_yz = [
        (-0.310, 0.000),
        (-0.308, 0.070),
        (-0.296, 0.620),
        (-0.280, 0.760),
        (-0.232, 0.900),
        (-0.175, 0.990),
        (-0.118, 1.300),
        (-0.026, 1.530),
        (0.232, 1.530),
        (0.286, 0.880),
        (0.308, 0.240),
        (0.310, 0.000),
    ]
    left = [(-width * 0.5, y, z) for y, z in side_profile_yz]
    right = [(width * 0.5, y, z) for y, z in side_profile_yz]
    return mesh_from_geometry(section_loft([left, right]), "casino_machine_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_casino_machine")

    cabinet_black = model.material("cabinet_black", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    metallic_trim = model.material("metallic_trim", rgba=(0.67, 0.69, 0.72, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.09, 0.16, 0.22, 0.55))
    deck_black = model.material("deck_black", rgba=(0.08, 0.09, 0.10, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.12, 0.10, 1.0))
    amber_lock = model.material("amber_lock", rgba=(0.70, 0.55, 0.18, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))

    cabinet_width = 0.740
    cabinet_depth = 0.620
    cabinet_height = 1.530

    screen_roll = 0.42
    deck_roll = 0.64
    screen_center = (0.0, -0.145, 1.190)
    deck_center = (0.0, -0.220, 0.870)

    door_width = 0.500
    door_height = 0.540
    door_thickness = 0.022
    door_bottom_z = 0.120
    hinge_x = -door_width * 0.5
    hinge_y = -0.326

    cabinet = model.part("cabinet")
    cabinet.visual(
        _cabinet_shell_mesh(cabinet_width),
        material=cabinet_black,
        name="body_shell",
    )
    cabinet.visual(
        Box((0.780, 0.680, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.610, 0.500, 0.040)),
        origin=Origin(xyz=screen_center, rpy=(screen_roll, 0.0, 0.0)),
        material=metallic_trim,
        name="screen_bezel",
    )
    cabinet.visual(
        Box((0.500, 0.380, 0.012)),
        origin=Origin(
            xyz=_panel_point(screen_center, screen_roll, 0.0, 0.0, 0.020),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=smoked_glass,
        name="screen_glass",
    )
    cabinet.visual(
        _button_deck_mesh(),
        origin=Origin(xyz=deck_center, rpy=(deck_roll, 0.0, 0.0)),
        material=deck_black,
        name="button_deck",
    )
    cabinet.visual(
        Box((0.660, 0.030, 0.018)),
        origin=Origin(
            xyz=_panel_point(deck_center, deck_roll, 0.0, -0.090, 0.014),
            rpy=(deck_roll, 0.0, 0.0),
        ),
        material=metallic_trim,
        name="deck_front_trim",
    )
    cabinet.visual(
        Box((0.560, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, -0.304, 0.085)),
        material=satin_charcoal,
        name="door_sill",
    )
    cabinet.visual(
        Box((0.024, 0.032, 0.600)),
        origin=Origin(xyz=(hinge_x - 0.025, hinge_y + 0.014, 0.390)),
        material=satin_charcoal,
        name="hinge_jamb",
    )
    cabinet.visual(
        Box((0.050, 0.042, 0.600)),
        origin=Origin(xyz=(0.0, -0.292, 0.390)),
        material=metallic_trim,
        name="door_header",
    )
    cabinet.visual(
        Box((0.470, 0.020, 0.470)),
        origin=Origin(xyz=(0.0, -0.205, 0.390)),
        material=dark_metal,
        name="service_bay_back",
    )
    cabinet.visual(
        Box((0.040, 0.110, 0.500)),
        origin=Origin(xyz=(-0.240, -0.250, 0.390)),
        material=dark_metal,
        name="service_bay_left_wall",
    )
    cabinet.visual(
        Box((0.040, 0.110, 0.500)),
        origin=Origin(xyz=(0.240, -0.250, 0.390)),
        material=dark_metal,
        name="service_bay_right_wall",
    )
    cabinet.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.231, -0.310, 0.390)),
        material=metallic_trim,
        name="door_strike_block",
    )
    cabinet.visual(
        Box((0.470, 0.110, 0.040)),
        origin=Origin(xyz=(0.0, -0.250, 0.140)),
        material=dark_metal,
        name="service_bay_floor",
    )
    cabinet.visual(
        Box((0.470, 0.110, 0.040)),
        origin=Origin(xyz=(0.0, -0.250, 0.640)),
        material=dark_metal,
        name="service_bay_ceiling",
    )

    hinge_radius = 0.013
    cabinet.visual(
        Cylinder(radius=hinge_radius, length=0.110),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.185)),
        material=dark_metal,
        name="upper_hinge_barrel",
    )
    cabinet.visual(
        Cylinder(radius=hinge_radius, length=0.110),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.595)),
        material=dark_metal,
        name="lower_hinge_barrel",
    )

    button_sleeve_origin = _panel_point(deck_center, deck_roll, 0.200, -0.010, 0.026)
    button_origin = _panel_point(deck_center, deck_roll, 0.200, -0.010, 0.050)
    knob_origin = _panel_point(screen_center, screen_roll, 0.285, 0.070, 0.026)

    button_sleeve_mesh = _ring_shell(
        "spin_button_sleeve",
        outer_radius=0.038,
        inner_radius=0.030,
        length=0.020,
    )
    cabinet.visual(
        button_sleeve_mesh,
        origin=Origin(xyz=button_sleeve_origin, rpy=(deck_roll, 0.0, 0.0)),
        material=metallic_trim,
        name="button_sleeve",
    )
    knob_bushing_mesh = _ring_shell(
        "volume_knob_bushing",
        outer_radius=0.022,
        inner_radius=0.0195,
        length=0.012,
    )
    cabinet.visual(
        knob_bushing_mesh,
        origin=Origin(xyz=knob_origin, rpy=(screen_roll, 0.0, 0.0)),
        material=metallic_trim,
        name="knob_bushing",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.780, cabinet_depth, cabinet_height)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    belly_door = model.part("belly_door")
    door_panel_clearance = 0.018
    door_panel_width = door_width - door_panel_clearance
    belly_door.visual(
        Box((door_panel_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_panel_clearance + door_panel_width * 0.5, 0.0, door_height * 0.5)
        ),
        material=satin_charcoal,
        name="door_panel",
    )
    belly_door.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, door_height * 0.5)),
        material=dark_metal,
        name="door_center_hinge_barrel",
    )
    belly_door.visual(
        Box((0.008, 0.010, 0.180)),
        origin=Origin(xyz=(0.016, 0.0, door_height * 0.5)),
        material=dark_metal,
        name="door_hinge_leaf",
    )
    belly_door.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(
            xyz=(door_panel_clearance + door_panel_width - 0.040, -0.012, door_height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=amber_lock,
        name="door_lock_cylinder",
    )
    belly_door.visual(
        Box((0.050, 0.004, 0.050)),
        origin=Origin(
            xyz=(door_panel_clearance + door_panel_width - 0.040, -0.016, door_height * 0.5)
        ),
        material=metallic_trim,
        name="door_lock_plate",
    )
    belly_door.inertial = Inertial.from_geometry(
        Box((door_panel_width, 0.040, door_height)),
        mass=8.0,
        origin=Origin(
            xyz=(door_panel_clearance + door_panel_width * 0.5, 0.0, door_height * 0.5)
        ),
    )

    spin_button = model.part("spin_button")
    spin_button.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=button_red,
        name="button_cap",
    )
    spin_button.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_metal,
        name="button_stem",
    )
    spin_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.068),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="knob_body",
    )
    volume_knob.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_metal,
        name="knob_shaft",
    )
    volume_knob.visual(
        Box((0.004, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.011, 0.024)),
        material=metallic_trim,
        name="knob_pointer",
    )
    volume_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.054),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "cabinet_to_belly_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=belly_door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "cabinet_to_spin_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=spin_button,
        origin=Origin(xyz=button_origin, rpy=(deck_roll, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=0.012,
        ),
    )
    model.articulation(
        "cabinet_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=volume_knob,
        origin=Origin(xyz=knob_origin, rpy=(screen_roll, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=8.0,
        ),
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

    cabinet = object_model.get_part("cabinet")
    belly_door = object_model.get_part("belly_door")
    spin_button = object_model.get_part("spin_button")
    volume_knob = object_model.get_part("volume_knob")

    door_hinge = object_model.get_articulation("cabinet_to_belly_door")
    button_slide = object_model.get_articulation("cabinet_to_spin_button")
    knob_spin = object_model.get_articulation("cabinet_to_volume_knob")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.check(
        "primary parts and articulations exist",
        all(
            item is not None
            for item in (
                cabinet,
                belly_door,
                spin_button,
                volume_knob,
                door_hinge,
                button_slide,
                knob_spin,
            )
        ),
    )

    ctx.allow_overlap(
        cabinet,
        spin_button,
        elem_a="body_shell",
        elem_b="button_stem",
        reason="The cabinet shell is authored as a closed exterior proxy, so the button stem is allowed to enter through the implied deck opening behind the trim sleeve.",
    )
    ctx.allow_overlap(
        cabinet,
        volume_knob,
        elem_a="screen_bezel",
        elem_b="knob_shaft",
        reason="The slanted screen bezel is modeled as a solid trim slab, so the volume knob shaft is allowed to pass through the implied mounting bore into the bushing.",
    )

    ctx.expect_overlap(
        belly_door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="service_bay_back",
        min_overlap=0.42,
        name="belly door covers the service bay opening",
    )

    closed_center = _aabb_center(ctx.part_element_world_aabb(belly_door, elem="door_panel"))
    with ctx.pose({door_hinge: 1.20}):
        opened_center = _aabb_center(ctx.part_element_world_aabb(belly_door, elem="door_panel"))
        ctx.expect_gap(
            cabinet,
            belly_door,
            axis="y",
            positive_elem="service_bay_back",
            negative_elem="door_panel",
            min_gap=0.015,
            name="opened belly door clears the cabinet interior plane",
        )
    ctx.check(
        "belly door swings outward on its side hinge",
        closed_center is not None
        and opened_center is not None
        and opened_center[1] < closed_center[1] - 0.080,
        details=f"closed_center={closed_center}, opened_center={opened_center}",
    )

    rest_button_pos = ctx.part_world_position(spin_button)
    with ctx.pose({button_slide: 0.012}):
        pressed_button_pos = ctx.part_world_position(spin_button)
    ctx.check(
        "spin button translates down into the deck sleeve",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.005
        and pressed_button_pos[2] < rest_button_pos[2] - 0.005,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    pointer_at_zero = _aabb_center(
        ctx.part_element_world_aabb(volume_knob, elem="knob_pointer")
    )
    with ctx.pose({knob_spin: math.pi * 0.5}):
        pointer_at_quarter_turn = _aabb_center(
            ctx.part_element_world_aabb(volume_knob, elem="knob_pointer")
        )
    ctx.check(
        "volume knob rotates continuously about its shaft",
        pointer_at_zero is not None
        and pointer_at_quarter_turn is not None
        and abs(pointer_at_quarter_turn[0] - pointer_at_zero[0]) > 0.010,
        details=(
            f"pointer_at_zero={pointer_at_zero}, "
            f"pointer_at_quarter_turn={pointer_at_quarter_turn}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
