from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_RADIUS = 0.19
BODY_DEPTH = 0.12
SHELL_WALL = 0.018
SHELL_LENGTH = 0.108
SHELL_CENTER_Y = 0.002
BACK_PANEL_RADIUS = 0.175
BACK_PANEL_THICKNESS = 0.008
BACK_PANEL_CENTER_Y = -0.056
BACK_PANEL_REAR_Y = BACK_PANEL_CENTER_Y - BACK_PANEL_THICKNESS / 2.0
BEZEL_OUTER_RADIUS = 0.174
BEZEL_INNER_RADIUS = 0.148
BEZEL_DEPTH = 0.012
GRILLE_Y = 0.048
CONTROL_PANEL_SIZE = (0.22, 0.018, 0.056)
CONTROL_PANEL_CENTER = (0.0, 0.051, -0.136)
BOSS_OUTER_RADIUS = 0.024
BOSS_INNER_RADIUS = 0.015
BOSS_DEPTH = 0.008
BOSS_CENTER_Y = 0.064
BOSS_SOCKET_THICKNESS = 0.001
BOSS_SOCKET_CENTER_Y = 0.0605
KNOB_XS = (-0.062, 0.0, 0.062)
KNOB_Z = -0.136
FOOT_RADIUS = 0.014
FOOT_DEPTH = 0.014
FOOT_Y = -0.067
FOOT_OFFSETS = {
    "back_upper_left": (-0.112, FOOT_Y, 0.112),
    "back_upper_right": (0.112, FOOT_Y, 0.112),
    "back_lower_left": (-0.112, FOOT_Y, -0.112),
    "back_lower_right": (0.112, FOOT_Y, -0.112),
}


def _annulus_mesh(filename: str, outer_radius: float, inner_radius: float, length: float):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_body_box_fan", assets=ASSETS)

    housing = model.material("housing_cream", rgba=(0.84, 0.82, 0.76, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    grille_metal = model.material("grille_metal", rgba=(0.67, 0.70, 0.73, 1.0))
    dark_socket = model.material("socket_shadow", rgba=(0.18, 0.18, 0.19, 1.0))
    knob_material = model.material("knob_charcoal", rgba=(0.14, 0.14, 0.16, 1.0))
    blade_material = model.material("blade_silver", rgba=(0.75, 0.77, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    shell_ring = _annulus_mesh("fan_shell_ring.obj", BODY_RADIUS, BODY_RADIUS - SHELL_WALL, SHELL_LENGTH)
    front_bezel = _annulus_mesh("fan_front_bezel.obj", BEZEL_OUTER_RADIUS, BEZEL_INNER_RADIUS, BEZEL_DEPTH)
    boss_flange = _annulus_mesh("fan_boss_flange.obj", BOSS_OUTER_RADIUS, BOSS_INNER_RADIUS, BOSS_DEPTH)

    body = model.part("body")
    body.visual(
        shell_ring,
        origin=Origin(xyz=(0.0, SHELL_CENTER_Y, 0.0)),
        material=housing,
        name="shell_ring",
    )
    body.visual(
        Cylinder(radius=BACK_PANEL_RADIUS, length=BACK_PANEL_THICKNESS),
        origin=Origin(xyz=(0.0, BACK_PANEL_CENTER_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="back_panel",
    )
    body.visual(
        front_bezel,
        origin=Origin(xyz=(0.0, 0.054, 0.0)),
        material=chrome,
        name="front_bezel",
    )
    body.visual(
        Box(CONTROL_PANEL_SIZE),
        origin=Origin(xyz=CONTROL_PANEL_CENTER),
        material=housing,
        name="control_panel",
    )

    for i, x in enumerate((-0.105, -0.052, 0.0, 0.052, 0.105)):
        body.visual(
            Box((0.006, 0.004, 0.30)),
            origin=Origin(xyz=(x, GRILLE_Y, 0.0)),
            material=grille_metal,
            name=f"grille_v{i}",
        )
    for i, z in enumerate((-0.090, -0.045, 0.0, 0.045, 0.090)):
        body.visual(
            Box((0.30, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, GRILLE_Y, z)),
            material=grille_metal,
            name=f"grille_h{i}",
        )

    knob_specs = (
        ("left", KNOB_XS[0]),
        ("center", KNOB_XS[1]),
        ("right", KNOB_XS[2]),
    )
    for knob_name, x in knob_specs:
        body.visual(
            boss_flange,
            origin=Origin(xyz=(x, BOSS_CENTER_Y, KNOB_Z)),
            material=housing,
            name=f"{knob_name}_boss_flange",
        )
        body.visual(
            Cylinder(radius=BOSS_INNER_RADIUS - 0.001, length=BOSS_SOCKET_THICKNESS),
            origin=Origin(
                xyz=(x, BOSS_SOCKET_CENTER_Y, KNOB_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_socket,
            name=f"{knob_name}_boss_socket",
        )
        body.visual(
            Box((0.003, 0.002, 0.010)),
            origin=Origin(xyz=(x, 0.069, KNOB_Z + 0.015)),
            material=chrome,
            name=f"{knob_name}_tick",
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_DEPTH),
        mass=3.6,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille_metal,
        name="motor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.062, length=0.036),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_material,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hub_cap",
    )
    blade_box = Box((0.11, 0.006, 0.045))
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            blade_box,
            origin=Origin(
                xyz=(0.086 * math.cos(angle), 0.051, 0.086 * math.sin(angle)),
                rpy=(0.18, -angle, 0.0),
            ),
            material=blade_material,
            name=f"blade_{i}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.04),
        mass=0.55,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    for knob_name in ("left", "center", "right"):
        knob = model.part(f"{knob_name}_knob")
        knob.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=grille_metal,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.0, 0.0155, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=knob_material,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.003, 0.010)),
            origin=Origin(xyz=(0.0, 0.0260, 0.010)),
            material=chrome,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.012, length=0.026),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0135, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )

    for foot_name, (x, y, z) in FOOT_OFFSETS.items():
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=FOOT_RADIUS, length=FOOT_DEPTH),
            origin=Origin(xyz=(0.0, -FOOT_DEPTH / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="pad",
        )
        foot.inertial = Inertial.from_geometry(
            Cylinder(radius=FOOT_RADIUS, length=FOOT_DEPTH),
            mass=0.03,
            origin=Origin(xyz=(0.0, -FOOT_DEPTH / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "body_to_rotor",
        ArticulationType.FIXED,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.052, 0.0)),
    )
    for knob_name, x in knob_specs:
        model.articulation(
            f"body_to_{knob_name}_knob",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=f"{knob_name}_knob",
            origin=Origin(xyz=(x, BOSS_SOCKET_CENTER_Y, KNOB_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=6.0),
        )
    for foot_name in FOOT_OFFSETS:
        x, _, z = FOOT_OFFSETS[foot_name]
        model.articulation(
            f"body_to_{foot_name}",
            ArticulationType.FIXED,
            parent=body,
            child=foot_name,
            origin=Origin(xyz=(x, BACK_PANEL_REAR_Y, z)),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    rotor = object_model.get_part("rotor")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")
    back_upper_left = object_model.get_part("back_upper_left")
    back_upper_right = object_model.get_part("back_upper_right")
    back_lower_left = object_model.get_part("back_lower_left")
    back_lower_right = object_model.get_part("back_lower_right")
    left_knob_spin = object_model.get_articulation("body_to_left_knob")

    back_panel = body.get_visual("back_panel")
    front_bezel = body.get_visual("front_bezel")
    control_panel = body.get_visual("control_panel")
    grille_v2 = body.get_visual("grille_v2")
    left_boss_socket = body.get_visual("left_boss_socket")
    center_boss_socket = body.get_visual("center_boss_socket")
    right_boss_socket = body.get_visual("right_boss_socket")
    left_tick = body.get_visual("left_tick")
    hub = rotor.get_visual("hub")
    motor_shaft = rotor.get_visual("motor_shaft")
    left_shaft = left_knob.get_visual("shaft")
    center_shaft = center_knob.get_visual("shaft")
    right_shaft = right_knob.get_visual("shaft")
    left_knob_body = left_knob.get_visual("knob_body")
    center_knob_body = center_knob.get_visual("knob_body")
    right_knob_body = right_knob.get_visual("knob_body")
    left_indicator = left_knob.get_visual("indicator")
    back_upper_left_pad = back_upper_left.get_visual("pad")
    back_upper_right_pad = back_upper_right.get_visual("pad")
    back_lower_left_pad = back_lower_left.get_visual("pad")
    back_lower_right_pad = back_lower_right.get_visual("pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_origin_distance(rotor, body, axes="xz", max_dist=0.001)
    ctx.expect_gap(
        body,
        rotor,
        axis="y",
        min_gap=0.016,
        positive_elem=grille_v2,
        negative_elem=hub,
        name="hub_sits_behind_front_grille",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="xz",
        elem_a=hub,
        elem_b=grille_v2,
        min_overlap=0.006,
        name="hub_reads_through_center_grille_bar",
    )
    ctx.expect_gap(
        rotor,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=motor_shaft,
        negative_elem=back_panel,
        name="rotor_mounts_to_back_panel",
    )

    ctx.expect_origin_distance(center_knob, body, axes="x", max_dist=0.001)
    ctx.expect_origin_distance(left_knob, right_knob, axes="z", max_dist=0.001)
    ctx.expect_overlap(
        body,
        body,
        axes="xz",
        elem_a=front_bezel,
        elem_b=grille_v2,
        min_overlap=0.006,
        name="chrome_bezel_frames_front_grille",
    )
    ctx.expect_gap(
        left_knob,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_shaft,
        negative_elem=left_boss_socket,
        name="left_knob_seats_on_recessed_socket",
    )
    ctx.expect_gap(
        center_knob,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=center_shaft,
        negative_elem=center_boss_socket,
        name="center_knob_seats_on_recessed_socket",
    )
    ctx.expect_gap(
        right_knob,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_shaft,
        negative_elem=right_boss_socket,
        name="right_knob_seats_on_recessed_socket",
    )
    ctx.expect_within(
        left_knob,
        body,
        axes="xz",
        inner_elem=left_knob_body,
        outer_elem=left_boss_socket,
    )
    ctx.expect_within(
        center_knob,
        body,
        axes="xz",
        inner_elem=center_knob_body,
        outer_elem=center_boss_socket,
    )
    ctx.expect_within(
        right_knob,
        body,
        axes="xz",
        inner_elem=right_knob_body,
        outer_elem=right_boss_socket,
    )
    ctx.expect_overlap(
        center_knob,
        body,
        axes="xz",
        elem_a=center_knob_body,
        elem_b=control_panel,
        min_overlap=0.02,
        name="center_knob_sits_on_lower_front_panel",
    )
    ctx.expect_gap(
        center_knob,
        body,
        axis="y",
        min_gap=0.006,
        max_gap=0.0085,
        positive_elem=center_knob_body,
        negative_elem=control_panel,
        name="center_knob_projects_from_front_panel_face",
    )
    ctx.expect_gap(
        center_knob,
        left_knob,
        axis="x",
        min_gap=0.036,
        max_gap=0.040,
        name="left_and_center_knobs_have_even_spacing",
    )
    ctx.expect_gap(
        right_knob,
        center_knob,
        axis="x",
        min_gap=0.036,
        max_gap=0.040,
        name="center_and_right_knobs_have_even_spacing",
    )
    ctx.expect_gap(
        rotor,
        center_knob,
        axis="z",
        min_gap=0.045,
        positive_elem=hub,
        negative_elem=center_knob_body,
        name="knob_row_sits_below_center_hub",
    )
    with ctx.pose({left_knob_spin: math.pi / 2.0}):
        ctx.expect_gap(
            left_knob,
            body,
            axis="x",
            min_gap=0.003,
            positive_elem=left_indicator,
            negative_elem=left_tick,
            name="left_knob_indicator_moves_off_top_tick_when_turned",
        )

    for foot_part, foot_pad, name in (
        (back_upper_left, back_upper_left_pad, "back_upper_left"),
        (back_upper_right, back_upper_right_pad, "back_upper_right"),
        (back_lower_left, back_lower_left_pad, "back_lower_left"),
        (back_lower_right, back_lower_right_pad, "back_lower_right"),
    ):
        ctx.expect_gap(
            body,
            foot_part,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=back_panel,
            negative_elem=foot_pad,
            name=f"{name}_foot_touches_back_panel",
        )
        ctx.expect_within(
            foot_part,
            body,
            axes="xz",
            inner_elem=foot_pad,
            outer_elem=back_panel,
            name=f"{name}_foot_stays_within_back_panel",
        )

    ctx.expect_gap(
        back_upper_left,
        back_lower_left,
        axis="z",
        min_gap=0.19,
        max_gap=0.20,
        name="left_feet_span_back_panel_height",
    )
    ctx.expect_gap(
        back_upper_right,
        back_upper_left,
        axis="x",
        min_gap=0.19,
        max_gap=0.20,
        name="upper_feet_span_back_panel_width",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
