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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_tower_crane", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.72, 1.0))
    crane_yellow = model.material("crane_yellow", rgba=(0.91, 0.76, 0.16, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    ballast_dark = model.material("ballast_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    cable_black = model.material("cable_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hook_red = model.material("hook_red", rgba=(0.73, 0.12, 0.10, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.18, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=concrete,
        name="foundation_pad",
    )
    tower.visual(
        Cylinder(radius=0.036, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=machinery_gray,
        name="mast_base",
    )
    tower.visual(
        Cylinder(radius=0.026, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=crane_yellow,
        name="mast",
    )
    tower.visual(
        Cylinder(radius=0.038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=machinery_gray,
        name="mast_head",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.316)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
    )

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.04, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=machinery_gray,
        name="turntable",
    )
    jib.visual(
        Box((0.082, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=machinery_gray,
        name="root_house",
    )
    jib.visual(
        Box((0.032, 0.044, 0.024)),
        origin=Origin(xyz=(-0.057, 0.0, 0.03)),
        material=ballast_dark,
        name="counterweight_box",
    )
    jib.visual(
        Box((0.202, 0.014, 0.012)),
        origin=Origin(xyz=(0.142, 0.0, 0.028)),
        material=crane_yellow,
        name="boom_lower_chord",
    )
    jib.visual(
        Box((0.164, 0.01, 0.008)),
        origin=Origin(xyz=(0.132, 0.0, 0.072)),
        material=crane_yellow,
        name="boom_upper_chord",
    )
    jib.visual(
        Box((0.01, 0.014, 0.034)),
        origin=Origin(xyz=(0.05, 0.0, 0.051)),
        material=crane_yellow,
        name="boom_post_root",
    )
    jib.visual(
        Box((0.008, 0.012, 0.034)),
        origin=Origin(xyz=(0.132, 0.0, 0.051)),
        material=crane_yellow,
        name="boom_post_mid",
    )
    jib.visual(
        Box((0.008, 0.012, 0.034)),
        origin=Origin(xyz=(0.214, 0.0, 0.051)),
        material=crane_yellow,
        name="boom_post_tip",
    )
    jib.visual(
        Cylinder(radius=0.002, length=0.0516),
        origin=Origin(xyz=(0.09, 0.0, 0.051), rpy=(0.0, 2.3996453855838755, 0.0)),
        material=crane_yellow,
        name="boom_brace_root",
    )
    jib.visual(
        Cylinder(radius=0.002, length=0.0933),
        origin=Origin(xyz=(0.174, 0.0, 0.051), rpy=(0.0, 2.649381386750227, 0.0)),
        material=crane_yellow,
        name="boom_brace_tip",
    )
    jib.visual(
        Box((0.02, 0.028, 0.024)),
        origin=Origin(xyz=(0.253, 0.0, 0.034)),
        material=machinery_gray,
        name="tip_block",
    )
    jib.visual(
        Box((0.01, 0.004, 0.018)),
        origin=Origin(xyz=(0.253, 0.009, 0.013)),
        material=machinery_gray,
        name="hook_hanger_left",
    )
    jib.visual(
        Box((0.01, 0.004, 0.018)),
        origin=Origin(xyz=(0.253, -0.009, 0.013)),
        material=machinery_gray,
        name="hook_hanger_right",
    )
    jib.inertial = Inertial.from_geometry(
        Box((0.322, 0.08, 0.1)),
        mass=1.6,
        origin=Origin(xyz=(0.096, 0.0, 0.05)),
    )

    hoist_drum = model.part("hoist_drum")
    hoist_drum.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="drum_body",
    )
    hoist_drum.visual(
        Box((0.004, 0.004, 0.004)),
        origin=Origin(xyz=(0.01, 0.01, 0.0)),
        material=ballast_dark,
        name="drum_key",
    )
    hoist_drum.inertial = Inertial.from_geometry(
        Box((0.03, 0.018, 0.03)),
        mass=0.18,
        origin=Origin(),
    )

    hook_assembly = model.part("hook_assembly")
    hook_assembly.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="pivot_pin",
    )
    hook_assembly.visual(
        Box((0.01, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=machinery_gray,
        name="hanger_body",
    )
    hook_assembly.visual(
        Cylinder(radius=0.002, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=cable_black,
        name="hoist_cable",
    )
    hook_assembly.visual(
        Box((0.014, 0.01, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        material=hook_red,
        name="hook_block",
    )
    hook_assembly.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.004, 0.0, -0.124), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hook_red,
        name="hook_tip",
    )
    hook_assembly.inertial = Inertial.from_geometry(
        Box((0.03, 0.018, 0.13)),
        mass=0.12,
        origin=Origin(xyz=(0.004, 0.0, -0.065)),
    )

    model.articulation(
        "slew_rotation",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 0.316)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.8,
            lower=-math.radians(160.0),
            upper=math.radians(160.0),
        ),
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=jib,
        child=hoist_drum,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=10.0,
        ),
    )

    model.articulation(
        "hook_sway",
        ArticulationType.REVOLUTE,
        parent=jib,
        child=hook_assembly,
        origin=Origin(xyz=(0.253, 0.0, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.2,
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    jib = object_model.get_part("jib")
    hoist_drum = object_model.get_part("hoist_drum")
    hook_assembly = object_model.get_part("hook_assembly")
    slew_rotation = object_model.get_articulation("slew_rotation")
    drum_spin = object_model.get_articulation("drum_spin")
    hook_sway = object_model.get_articulation("hook_sway")

    foundation_pad = tower.get_visual("foundation_pad")
    mast = tower.get_visual("mast")
    mast_head = tower.get_visual("mast_head")
    turntable = jib.get_visual("turntable")
    root_house = jib.get_visual("root_house")
    counterweight_box = jib.get_visual("counterweight_box")
    boom_lower_chord = jib.get_visual("boom_lower_chord")
    boom_upper_chord = jib.get_visual("boom_upper_chord")
    boom_post_tip = jib.get_visual("boom_post_tip")
    tip_block = jib.get_visual("tip_block")
    drum_key = hoist_drum.get_visual("drum_key")
    drum_body = hoist_drum.get_visual("drum_body")
    hoist_cable = hook_assembly.get_visual("hoist_cable")
    hook_block = hook_assembly.get_visual("hook_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_within(tower, tower, axes="xy", inner_elem=mast, outer_elem=foundation_pad)
    ctx.expect_origin_distance(jib, tower, axes="xy", max_dist=0.001)
    ctx.expect_within(tower, jib, axes="xy", inner_elem=mast_head, outer_elem=turntable, margin=0.002)
    ctx.expect_contact(jib, tower, elem_a=turntable, elem_b=mast_head)
    ctx.expect_contact(hoist_drum, jib, elem_a=drum_body)
    ctx.expect_contact(hook_assembly, jib)
    ctx.expect_gap(
        jib,
        hook_assembly,
        axis="z",
        min_gap=0.11,
        max_gap=0.14,
        positive_elem=tip_block,
        negative_elem=hook_block,
    )
    ctx.expect_overlap(jib, jib, axes="x", elem_a=boom_upper_chord, elem_b=boom_post_tip, min_overlap=0.0035)
    ctx.expect_contact(jib, jib, elem_a=counterweight_box, elem_b=root_house)
    ctx.expect_overlap(hook_assembly, jib, axes="x", elem_a=hoist_cable, elem_b=tip_block, min_overlap=0.004)

    ctx.check(
        "slew_axis_vertical",
        tuple(slew_rotation.axis) == (0.0, 0.0, 1.0),
        f"expected vertical slew axis, got {slew_rotation.axis}",
    )
    ctx.check(
        "drum_axis_transverse",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        f"expected transverse drum axis, got {drum_spin.axis}",
    )
    ctx.check(
        "hook_axis_transverse",
        tuple(hook_sway.axis) == (0.0, 1.0, 0.0),
        f"expected hook sway axis, got {hook_sway.axis}",
    )

    hook_origin_rest = ctx.part_world_position(hook_assembly)
    drum_key_rest = _aabb_center(ctx.part_element_world_aabb(hoist_drum, elem=drum_key))
    hook_block_rest = _aabb_center(ctx.part_element_world_aabb(hook_assembly, elem=hook_block))

    ctx.check("hook_origin_available", hook_origin_rest is not None, "hook origin should resolve in rest pose")
    ctx.check("drum_key_available", drum_key_rest is not None, "drum key AABB should resolve in rest pose")
    ctx.check("hook_block_available", hook_block_rest is not None, "hook block AABB should resolve in rest pose")

    if hook_origin_rest is not None:
        with ctx.pose({slew_rotation: math.pi / 2.0}):
            hook_origin_slewed = ctx.part_world_position(hook_assembly)
            ctx.expect_contact(jib, tower, elem_a=turntable, elem_b=mast_head)
            ctx.expect_contact(hoist_drum, jib, elem_a=drum_body)
            ctx.check(
                "slew_rotates_jib_planform",
                hook_origin_slewed is not None
                and abs(hook_origin_slewed[0]) < 0.03
                and hook_origin_slewed[1] > hook_origin_rest[0] - 0.03,
                f"expected hook pivot to swing from +x toward +y, got {hook_origin_slewed}",
            )

    if drum_key_rest is not None:
        with ctx.pose({drum_spin: math.pi / 2.0}):
            drum_key_turned = _aabb_center(ctx.part_element_world_aabb(hoist_drum, elem=drum_key))
            ctx.expect_contact(hoist_drum, jib, elem_a=drum_body)
            ctx.check(
                "drum_spin_moves_keyed_feature",
                drum_key_turned is not None
                and abs(drum_key_turned[0]) < 0.0025
                and abs(drum_key_turned[2] - drum_key_rest[2]) > 0.008,
                f"expected keyed feature to rotate away from +x, got {drum_key_turned}",
            )

    if hook_block_rest is not None and hook_sway.motion_limits is not None:
        limits = hook_sway.motion_limits
        if limits.lower is not None:
            with ctx.pose({hook_sway: limits.lower}):
                hook_block_swung = _aabb_center(ctx.part_element_world_aabb(hook_assembly, elem=hook_block))
                ctx.expect_contact(hook_assembly, jib)
                ctx.check(
                    "hook_sway_moves_block_forward",
                    hook_block_swung is not None and hook_block_swung[0] > hook_block_rest[0] + 0.03,
                    f"expected hook block to move forward under sway, got {hook_block_swung}",
                )
                ctx.check(
                    "hook_sway_raises_block",
                    hook_block_swung is not None and hook_block_swung[2] > hook_block_rest[2] + 0.01,
                    f"expected hook block to rise while swinging, got {hook_block_swung}",
                )
        if limits.upper is not None:
            with ctx.pose({hook_sway: limits.upper}):
                hook_block_swung = _aabb_center(ctx.part_element_world_aabb(hook_assembly, elem=hook_block))
                ctx.expect_contact(hook_assembly, jib)
                ctx.check(
                    "hook_sway_moves_block_aft",
                    hook_block_swung is not None and hook_block_swung[0] < hook_block_rest[0] - 0.03,
                    f"expected hook block to move aft under opposite sway, got {hook_block_swung}",
                )

    for articulated_joint in (slew_rotation, hook_sway):
        limits = articulated_joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulated_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_lower_no_floating")
        with ctx.pose({articulated_joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
