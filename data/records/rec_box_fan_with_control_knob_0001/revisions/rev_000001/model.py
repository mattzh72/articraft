from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


IVORY_PLASTIC = Material("ivory_plastic", (0.90, 0.88, 0.83, 1.0))
GRAPHITE_PLASTIC = Material("graphite_plastic", (0.12, 0.13, 0.14, 1.0))
STEEL_WIRE = Material("steel_wire", (0.33, 0.35, 0.38, 1.0))
SMOKE_BLADE = Material("smoke_blade", (0.28, 0.32, 0.35, 0.58))
RUBBER = Material("rubber", (0.08, 0.08, 0.09, 1.0))
SILVER = Material("silver_trim", (0.72, 0.73, 0.75, 1.0))
POINTER_WHITE = Material("pointer_white", (0.96, 0.96, 0.94, 1.0))


def _blade_section(chord: float, thickness: float, sweep: float, radius: float, twist: float):
    profile_2d = [
        (-0.48 * chord, -0.18 * thickness),
        (-0.20 * chord, -0.56 * thickness),
        (0.16 * chord, -0.44 * thickness),
        (0.50 * chord, 0.0),
        (0.18 * chord, 0.56 * thickness),
        (-0.42 * chord, 0.22 * thickness),
    ]
    c = math.cos(twist)
    s = math.sin(twist)
    section = []
    for x, y in profile_2d:
        x += sweep
        xr = c * x - s * y
        yr = s * x + c * y
        section.append((xr, yr, radius))
    return section


def _build_blade_mesh():
    blade_geom = LoftGeometry(
        [
            _blade_section(chord=0.088, thickness=0.011, sweep=0.006, radius=0.045, twist=0.32),
            _blade_section(chord=0.070, thickness=0.0085, sweep=0.014, radius=0.108, twist=0.14),
            _blade_section(chord=0.048, thickness=0.0060, sweep=0.025, radius=0.172, twist=-0.08),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(blade_geom, ASSETS.mesh_path("box_fan_blade.obj"))


def _add_grille(part, y: float, grille_w: float, grille_h: float, material: Material) -> None:
    frame_radius = 0.0035
    wire_radius = 0.0023

    part.visual(
        Cylinder(radius=frame_radius, length=grille_h),
        origin=Origin(xyz=(-grille_w / 2, y, 0.0)),
        material=material,
        name=f"grille_side_left_{y:+.3f}",
    )
    part.visual(
        Cylinder(radius=frame_radius, length=grille_h),
        origin=Origin(xyz=(grille_w / 2, y, 0.0)),
        material=material,
        name=f"grille_side_right_{y:+.3f}",
    )
    part.visual(
        Cylinder(radius=frame_radius, length=grille_w),
        origin=Origin(xyz=(0.0, y, grille_h / 2), rpy=(0.0, math.pi / 2, 0.0)),
        material=material,
        name=f"grille_top_{y:+.3f}",
    )
    part.visual(
        Cylinder(radius=frame_radius, length=grille_w),
        origin=Origin(xyz=(0.0, y, -grille_h / 2), rpy=(0.0, math.pi / 2, 0.0)),
        material=material,
        name=f"grille_bottom_{y:+.3f}",
    )

    vertical_count = 11
    horizontal_count = 11
    for i in range(vertical_count):
        x = -0.17 + i * (0.34 / (vertical_count - 1))
        part.visual(
            Cylinder(radius=wire_radius, length=grille_h - 0.014),
            origin=Origin(xyz=(x, y, 0.0)),
            material=material,
            name=f"grille_v_{i}_{y:+.3f}",
        )
    for i in range(horizontal_count):
        z = -0.17 + i * (0.34 / (horizontal_count - 1))
        part.visual(
            Cylinder(radius=wire_radius, length=grille_w - 0.014),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=material,
            name=f"grille_h_{i}_{y:+.3f}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan", assets=ASSETS)

    blade_mesh = _build_blade_mesh()

    outer_w = 0.52
    outer_h = 0.54
    depth = 0.16
    frame_t = 0.05
    corner_r = frame_t / 2
    front_y = depth / 2 - 0.012
    rear_y = -depth / 2 + 0.012

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((outer_w, depth, outer_h)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    side_h = outer_h - 2 * corner_r
    top_w = outer_w - 2 * corner_r
    for x in (-outer_w / 2 + frame_t / 2, outer_w / 2 - frame_t / 2):
        housing.visual(
            Box((frame_t, depth, side_h)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=IVORY_PLASTIC,
            name=f"side_column_{x:+.3f}",
        )
    for z in (-outer_h / 2 + frame_t / 2, outer_h / 2 - frame_t / 2):
        housing.visual(
            Box((top_w, depth, frame_t)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=IVORY_PLASTIC,
            name=f"cross_rail_{z:+.3f}",
        )
    for x in (-outer_w / 2 + corner_r, outer_w / 2 - corner_r):
        for z in (-outer_h / 2 + corner_r, outer_h / 2 - corner_r):
            housing.visual(
                Cylinder(radius=corner_r, length=depth),
                origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=IVORY_PLASTIC,
                name=f"corner_{x:+.3f}_{z:+.3f}",
            )

    for x in (-0.11, 0.11):
        housing.visual(
            Box((0.11, depth + 0.03, 0.018)),
            origin=Origin(xyz=(x, 0.0, -outer_h / 2 - 0.009)),
            material=IVORY_PLASTIC,
            name=f"foot_{x:+.3f}",
        )
        housing.visual(
            Box((0.09, depth + 0.02, 0.004)),
            origin=Origin(xyz=(x, 0.0, -outer_h / 2 - 0.020)),
            material=RUBBER,
            name=f"foot_pad_{x:+.3f}",
        )

    for x in (-0.105, 0.105):
        housing.visual(
            Box((0.028, depth * 0.80, 0.055)),
            origin=Origin(xyz=(x, 0.0, outer_h / 2 + 0.028)),
            material=IVORY_PLASTIC,
            name=f"handle_post_{x:+.3f}",
        )
    housing.visual(
        Box((0.235, depth * 0.64, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2 + 0.068)),
        material=IVORY_PLASTIC,
        name="handle_bridge",
    )
    housing.visual(
        Cylinder(radius=0.014, length=depth * 0.64),
        origin=Origin(xyz=(-0.1175, 0.0, outer_h / 2 + 0.068), rpy=(math.pi / 2, 0.0, 0.0)),
        material=IVORY_PLASTIC,
        name="handle_round_left",
    )
    housing.visual(
        Cylinder(radius=0.014, length=depth * 0.64),
        origin=Origin(xyz=(0.1175, 0.0, outer_h / 2 + 0.068), rpy=(math.pi / 2, 0.0, 0.0)),
        material=IVORY_PLASTIC,
        name="handle_round_right",
    )

    _add_grille(housing, front_y, grille_w=0.424, grille_h=0.424, material=STEEL_WIRE)
    _add_grille(housing, rear_y, grille_w=0.424, grille_h=0.424, material=STEEL_WIRE)

    housing.visual(
        Cylinder(radius=0.058, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=GRAPHITE_PLASTIC,
        name="rear_motor_cap",
    )
    housing.visual(
        Box((0.300, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=STEEL_WIRE,
        name="rear_support_horizontal",
    )
    housing.visual(
        Box((0.008, 0.008, 0.300)),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=STEEL_WIRE,
        name="rear_support_vertical",
    )
    housing.visual(
        Box((0.060, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, front_y, -0.205)),
        material=GRAPHITE_PLASTIC,
        name="front_badge",
    )

    control_pod = model.part("control_pod")
    control_pod.inertial = Inertial.from_geometry(
        Box((0.074, 0.090, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    control_pod.visual(
        Box((0.074, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=IVORY_PLASTIC,
        name="pod_body",
    )
    control_pod.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=SILVER,
        name="pod_knob_bezel",
    )
    control_pod.visual(
        Box((0.048, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, -0.031, 0.005)),
        material=GRAPHITE_PLASTIC,
        name="pod_label_strip",
    )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.028),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
    )
    rotor.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=GRAPHITE_PLASTIC,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=SILVER,
        name="hub_collar",
    )
    rotor.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=SILVER,
        name="spinner_cap",
    )
    for i in range(5):
        angle = i * (2.0 * math.pi / 5.0)
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(0.0, angle, 0.0)),
            material=SMOKE_BLADE,
            name=f"blade_{i}",
        )

    knob = model.part("knob")
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.012),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=GRAPHITE_PLASTIC,
        name="knob_base",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=GRAPHITE_PLASTIC,
        name="knob_cap",
    )
    knob.visual(
        Box((0.013, 0.003, 0.004)),
        origin=Origin(xyz=(0.0105, 0.0, 0.030)),
        material=POINTER_WHITE,
        name="knob_pointer",
    )

    model.articulation(
        "control_pod_mount",
        ArticulationType.FIXED,
        parent="housing",
        child="control_pod",
        origin=Origin(xyz=(0.155, 0.0, outer_h / 2)),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent="housing",
        child="rotor",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=28.0,
        ),
    )
    model.articulation(
        "speed_knob",
        ArticulationType.REVOLUTE,
        parent="control_pod",
        child="knob",
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.5,
            lower=0.0,
            upper=4.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("rotor", "housing", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("rotor", "housing", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("control_pod", "housing", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("knob", "control_pod", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_gap("knob", "control_pod", axis="z", max_gap=0.003, max_penetration=0.0)

    control_pos = ctx.part_world_position("control_pod")
    rotor_pos = ctx.part_world_position("rotor")
    knob_pos = ctx.part_world_position("knob")
    assert abs(rotor_pos[0]) < 1e-6 and abs(rotor_pos[1]) < 1e-6 and abs(rotor_pos[2]) < 1e-6
    assert control_pos[0] > 0.12
    assert control_pos[2] > 0.26
    assert knob_pos[0] > 0.12
    assert knob_pos[2] > control_pos[2]

    for angle in (0.0, math.pi / 2, math.pi, 3.0 * math.pi / 2):
        with ctx.pose(blade_spin=angle):
            ctx.expect_origin_distance("rotor", "housing", axes="xy", max_dist=0.01)
            ctx.expect_aabb_overlap("rotor", "housing", axes="xy", min_overlap=0.02)
            posed_rotor = ctx.part_world_position("rotor")
            assert math.dist(rotor_pos, posed_rotor) < 1e-9

    for angle in (0.0, 2.1, 4.2):
        with ctx.pose(speed_knob=angle):
            ctx.expect_aabb_overlap("knob", "control_pod", axes="xy", min_overlap=0.02)
            ctx.expect_aabb_gap("knob", "control_pod", axis="z", max_gap=0.003, max_penetration=0.0)
            posed_knob = ctx.part_world_position("knob")
            assert math.dist(knob_pos, posed_knob) < 1e-9

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
