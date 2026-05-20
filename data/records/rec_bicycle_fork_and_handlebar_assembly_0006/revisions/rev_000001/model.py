from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_suspension_fork", assets=ASSETS)

    alloy = model.material("alloy", rgba=(0.75, 0.77, 0.80, 1.0))
    stem_black = model.material("stem_black", rgba=(0.12, 0.13, 0.14, 1.0))
    lower_black = model.material("lower_black", rgba=(0.15, 0.16, 0.18, 1.0))
    stanchion_gold = model.material("stanchion_gold", rgba=(0.72, 0.58, 0.20, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hardware = model.material("hardware", rgba=(0.24, 0.25, 0.27, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    crown = model.part("crown_assembly")
    crown.inertial = Inertial.from_geometry(
        Box((0.30, 0.10, 0.48)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )
    crown.visual(
        Box((0.24, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=alloy,
        name="crown_plate",
    )
    crown.visual(
        Box((0.10, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=alloy,
        name="crown_center_block",
    )
    crown.visual(
        Box((0.08, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=alloy,
        name="underslung_web",
    )
    crown.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(0.07, 0.0, 0.18)),
        material=alloy,
        name="left_socket",
    )
    crown.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(-0.07, 0.0, 0.18)),
        material=alloy,
        name="right_socket",
    )
    crown.visual(
        Cylinder(radius=0.0175, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=alloy,
        name="steerer_tube",
    )
    crown.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.469)),
        material=hardware,
        name="top_cap",
    )

    stem = model.part("direct_mount_stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.12)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.035, 0.06)),
    )
    stem.visual(
        Box((0.028, 0.045, 0.018)),
        origin=Origin(xyz=(0.024, 0.010, 0.009)),
        material=stem_black,
        name="left_mount_foot",
    )
    stem.visual(
        Box((0.028, 0.045, 0.018)),
        origin=Origin(xyz=(-0.024, 0.010, 0.009)),
        material=stem_black,
        name="right_mount_foot",
    )
    stem.visual(
        Box((0.088, 0.084, 0.050)),
        origin=Origin(xyz=(0.0, 0.027, 0.034)),
        material=stem_black,
        name="stem_body",
    )
    stem.visual(
        Box((0.050, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.052, 0.050)),
        material=stem_black,
        name="lower_clamp",
    )
    stem.visual(
        Box((0.022, 0.022, 0.012)),
        origin=Origin(xyz=(0.016, 0.052, 0.094)),
        material=stem_black,
        name="upper_clamp_left",
    )
    stem.visual(
        Box((0.022, 0.022, 0.012)),
        origin=Origin(xyz=(-0.016, 0.052, 0.094)),
        material=stem_black,
        name="upper_clamp_right",
    )
    stem.visual(
        Box((0.060, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.068, 0.074)),
        material=stem_black,
        name="faceplate",
    )
    for x_sign in (-1.0, 1.0):
        for z_pos in (0.056, 0.090):
            stem.visual(
                Cylinder(radius=0.0045, length=0.014),
                origin=Origin(xyz=(0.021 * x_sign, 0.074, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
                material=hardware,
            )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.86, 0.11, 0.13)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.04, 0.08)),
    )
    bar_mesh = _save_mesh(
        "fork_handlebar.obj",
        tube_from_spline_points(
            [
                (-0.37, -0.036, 0.022),
                (-0.28, -0.028, 0.018),
                (-0.18, -0.014, 0.010),
                (-0.08, 0.0, 0.0),
                (0.08, 0.0, 0.0),
                (0.18, -0.014, 0.010),
                (0.28, -0.028, 0.018),
                (0.37, -0.036, 0.022),
            ],
            radius=0.0145,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    handlebar.visual(bar_mesh, material=stem_black, name="bar_span")
    handlebar.visual(
        Cylinder(radius=0.016, length=0.11),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stem_black,
        name="bar_core",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.37, -0.036, 0.022), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.37, -0.036, 0.022), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    def _build_leg(part_name: str, inboard_sign: float) -> None:
        leg = model.part(part_name)
        leg.inertial = Inertial.from_geometry(
            Box((0.08, 0.07, 0.62)),
            mass=1.6,
            origin=Origin(xyz=(0.0, 0.0, -0.31)),
        )
        leg.visual(
            Cylinder(radius=0.019, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=hardware,
            name="crown_seat",
        )
        leg.visual(
            Cylinder(radius=0.0175, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, -0.09)),
            material=stanchion_gold,
            name="stanchion_tube",
        )
        leg.visual(
            Cylinder(radius=0.024, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.193)),
            material=hardware,
            name="wiper_seal",
        )
        leg.visual(
            Cylinder(radius=0.026, length=0.37),
            origin=Origin(xyz=(0.0, 0.0, -0.391)),
            material=lower_black,
            name="lower_slider",
        )
        leg.visual(
            Box((0.040, 0.030, 0.055)),
            origin=Origin(xyz=(0.012 * inboard_sign, 0.0, -0.603)),
            material=lower_black,
            name="axle_dropout",
        )
        leg.visual(
            Box((0.012, 0.028, 0.022)),
            origin=Origin(xyz=(-0.022 * inboard_sign, 0.022, -0.430)),
            material=hardware,
            name="brake_mount",
        )

    _build_leg("left_stanchion_leg", inboard_sign=-1.0)
    _build_leg("right_stanchion_leg", inboard_sign=1.0)

    model.articulation(
        "stem_mount",
        ArticulationType.FIXED,
        parent="crown_assembly",
        child="direct_mount_stem",
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
    )
    model.articulation(
        "bar_clamp",
        ArticulationType.FIXED,
        parent="direct_mount_stem",
        child="handlebar",
        origin=Origin(xyz=(0.0, 0.052, 0.072)),
    )
    model.articulation(
        "left_leg_travel",
        ArticulationType.PRISMATIC,
        parent="crown_assembly",
        child="left_stanchion_leg",
        origin=Origin(xyz=(0.07, 0.0, 0.16)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2, lower=0.0, upper=0.09),
    )
    model.articulation(
        "right_leg_travel",
        ArticulationType.PRISMATIC,
        parent="crown_assembly",
        child="right_stanchion_leg",
        origin=Origin(xyz=(-0.07, 0.0, 0.16)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2, lower=0.0, upper=0.09),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    crown = object_model.get_part("crown_assembly")
    stem = object_model.get_part("direct_mount_stem")
    handlebar = object_model.get_part("handlebar")
    left_leg = object_model.get_part("left_stanchion_leg")
    right_leg = object_model.get_part("right_stanchion_leg")
    left_leg_travel = object_model.get_articulation("left_leg_travel")
    right_leg_travel = object_model.get_articulation("right_leg_travel")

    crown_center_block = crown.get_visual("crown_center_block")
    left_socket = crown.get_visual("left_socket")
    right_socket = crown.get_visual("right_socket")
    left_crown_seat = left_leg.get_visual("crown_seat")
    right_crown_seat = right_leg.get_visual("crown_seat")
    left_mount_foot = stem.get_visual("left_mount_foot")
    right_mount_foot = stem.get_visual("right_mount_foot")
    lower_clamp = stem.get_visual("lower_clamp")
    upper_clamp_left = stem.get_visual("upper_clamp_left")
    upper_clamp_right = stem.get_visual("upper_clamp_right")
    bar_core = handlebar.get_visual("bar_core")
    left_grip = handlebar.get_visual("left_grip")
    right_grip = handlebar.get_visual("right_grip")

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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    ctx.expect_gap(
        crown,
        left_leg,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_socket,
        negative_elem=left_crown_seat,
        name="left_leg_seats_into_crown",
    )
    ctx.expect_gap(
        crown,
        right_leg,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_socket,
        negative_elem=right_crown_seat,
        name="right_leg_seats_into_crown",
    )
    ctx.expect_overlap(left_leg, crown, axes="xy", min_overlap=0.001)
    ctx.expect_overlap(right_leg, crown, axes="xy", min_overlap=0.001)
    ctx.expect_origin_distance(left_leg, right_leg, axes="yz", max_dist=0.001)
    ctx.expect_overlap(left_leg, right_leg, axes="yz", min_overlap=0.05)

    ctx.expect_gap(
        stem,
        crown,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_mount_foot,
        negative_elem=crown_center_block,
        name="stem_left_foot_seated_on_crown",
    )
    ctx.expect_gap(
        stem,
        crown,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_mount_foot,
        negative_elem=crown_center_block,
        name="stem_right_foot_seated_on_crown",
    )
    ctx.expect_within(stem, crown, axes="x")
    ctx.expect_origin_distance(stem, crown, axes="x", max_dist=0.002)
    ctx.expect_contact(handlebar, stem, elem_a=bar_core, elem_b=lower_clamp)
    ctx.expect_contact(handlebar, stem, elem_a=bar_core, elem_b=upper_clamp_left)
    ctx.expect_contact(handlebar, stem, elem_a=bar_core, elem_b=upper_clamp_right)
    ctx.expect_origin_distance(handlebar, stem, axes="x", max_dist=0.002)
    ctx.expect_gap(
        handlebar,
        stem,
        axis="z",
        min_gap=0.015,
        positive_elem=left_grip,
        negative_elem=lower_clamp,
        name="left_grip_has_rise_above_stem",
    )
    ctx.expect_gap(
        handlebar,
        stem,
        axis="z",
        min_gap=0.015,
        positive_elem=right_grip,
        negative_elem=lower_clamp,
        name="right_grip_has_rise_above_stem",
    )
    ctx.expect_gap(
        handlebar,
        crown,
        axis="x",
        min_gap=0.18,
        positive_elem=right_grip,
        name="right_grip_extends_beyond_fork_width",
    )
    ctx.expect_gap(
        crown,
        handlebar,
        axis="x",
        min_gap=0.18,
        negative_elem=left_grip,
        name="left_grip_extends_beyond_fork_width",
    )
    ctx.expect_overlap(stem, crown, axes="xy", min_overlap=0.003)
    ctx.expect_overlap(handlebar, stem, axes="xy", min_overlap=0.01)

    with ctx.pose({left_leg_travel: 0.06, right_leg_travel: 0.06}):
        ctx.expect_gap(
            crown,
            left_leg,
            axis="z",
            min_gap=0.058,
            max_gap=0.062,
            positive_elem=left_socket,
            negative_elem=left_crown_seat,
            name="left_leg_extends_below_crown",
        )
        ctx.expect_gap(
            crown,
            right_leg,
            axis="z",
            min_gap=0.058,
            max_gap=0.062,
            positive_elem=right_socket,
            negative_elem=right_crown_seat,
            name="right_leg_extends_below_crown",
        )
        ctx.expect_overlap(left_leg, crown, axes="xy", min_overlap=0.001)
        ctx.expect_overlap(right_leg, crown, axes="xy", min_overlap=0.001)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
