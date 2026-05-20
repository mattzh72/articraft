from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.0055
EAR_T = 0.005
CLEVIS_GAP = 0.008
EAR_CENTER_Y = CLEVIS_GAP / 2.0 + EAR_T / 2.0
WASHER_T = 0.0012
PIN_HOLE_R = 0.0044
PIN_R = 0.0037
PIN_HEAD_R = 0.0086
PIN_SPAN = CLEVIS_GAP + 2.0 * EAR_T

ROOT_H = 0.055
BAR1_LEN = 0.112
BAR2_LEN = 0.160
BAR3_LEN = 0.096


def _xzy_cylinder(radius: float, length: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate((x, y, z))


def _extrude_xz_polygon(points: list[tuple[float, float]], thickness: float, *, y_center: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness / 2.0, both=True).translate((0.0, y_center, 0.0))


def _make_anchor_foot() -> cq.Workplane:
    foot_plate = cq.Workplane("XY").box(0.082, 0.054, 0.008).translate((-0.026, 0.0, 0.004))
    rear_block = cq.Workplane("XY").box(0.018, 0.026, 0.016).translate((-0.022, 0.0, 0.016))
    riser_left = cq.Workplane("XY").box(0.016, 0.008, 0.024).translate((-0.010, 0.0105, 0.024))
    riser_right = cq.Workplane("XY").box(0.016, 0.008, 0.024).translate((-0.010, -0.0105, 0.024))
    ear_left = cq.Workplane("XZ").rect(0.024, 0.032).extrude(EAR_T / 2.0, both=True).translate((-0.002, EAR_CENTER_Y, 0.044))
    ear_right = cq.Workplane("XZ").rect(0.024, 0.032).extrude(EAR_T / 2.0, both=True).translate((-0.002, -EAR_CENTER_Y, 0.044))
    root_pin = _xzy_cylinder(PIN_R, PIN_SPAN, x=0.0, y=0.0, z=ROOT_H)
    root_head_left = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=0.0, y=PIN_SPAN / 2.0 + WASHER_T / 2.0, z=ROOT_H)
    root_head_right = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=0.0, y=-PIN_SPAN / 2.0 - WASHER_T / 2.0, z=ROOT_H)

    shape = (
        foot_plate.union(rear_block)
        .union(riser_left)
        .union(riser_right)
        .union(ear_left)
        .union(ear_right)
        .union(root_pin)
        .union(root_head_left)
        .union(root_head_right)
    )

    mount_a = cq.Workplane("XY").circle(0.0042).extrude(0.012).translate((-0.048, 0.0, -0.002))
    mount_b = cq.Workplane("XY").circle(0.0042).extrude(0.012).translate((-0.010, 0.0, -0.002))
    mount_ca = cq.Workplane("XY").circle(0.0074).extrude(0.0026).translate((-0.048, 0.0, 0.0054))
    mount_cb = cq.Workplane("XY").circle(0.0074).extrude(0.0026).translate((-0.010, 0.0, 0.0054))
    return shape.cut(mount_a).cut(mount_b).cut(mount_ca).cut(mount_cb)


def _make_bar_one() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.072, PLATE_T, 0.020).translate((0.046, 0.0, 0.0))
    rear_eye = _xzy_cylinder(0.0135, PLATE_T, x=0.0, y=0.0, z=0.0)
    left_shoulder = cq.Workplane("XY").box(0.028, 0.0065, 0.012).translate((0.076, 0.0050, 0.0))
    right_shoulder = cq.Workplane("XY").box(0.028, 0.0065, 0.012).translate((0.076, -0.0050, 0.0))
    ear_left = cq.Workplane("XZ").rect(0.040, 0.028).extrude(EAR_T / 2.0, both=True).translate((0.092, EAR_CENTER_Y, 0.0))
    ear_right = cq.Workplane("XZ").rect(0.040, 0.028).extrude(EAR_T / 2.0, both=True).translate((0.092, -EAR_CENTER_Y, 0.0))
    front_pin = _xzy_cylinder(PIN_R, PIN_SPAN, x=BAR1_LEN, y=0.0, z=0.0)
    front_head_left = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=BAR1_LEN, y=PIN_SPAN / 2.0 + WASHER_T / 2.0, z=0.0)
    front_head_right = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=BAR1_LEN, y=-PIN_SPAN / 2.0 - WASHER_T / 2.0, z=0.0)

    shape = (
        beam.union(rear_eye)
        .union(left_shoulder)
        .union(right_shoulder)
        .union(ear_left)
        .union(ear_right)
        .union(front_pin)
        .union(front_head_left)
        .union(front_head_right)
    )
    rear_bore = _xzy_cylinder(PIN_HOLE_R, PIN_SPAN + 0.010, x=0.0, y=0.0, z=0.0)
    return shape.cut(rear_bore)


def _make_bar_two() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.108, PLATE_T, 0.018).translate((0.065, 0.0, 0.0))
    rear_eye = _xzy_cylinder(0.013, PLATE_T, x=0.0, y=0.0, z=0.0)
    left_shoulder = cq.Workplane("XY").box(0.034, 0.0065, 0.011).translate((0.121, 0.0050, 0.0))
    right_shoulder = cq.Workplane("XY").box(0.034, 0.0065, 0.011).translate((0.121, -0.0050, 0.0))
    ear_left = cq.Workplane("XZ").rect(0.044, 0.026).extrude(EAR_T / 2.0, both=True).translate((0.138, EAR_CENTER_Y, 0.0))
    ear_right = cq.Workplane("XZ").rect(0.044, 0.026).extrude(EAR_T / 2.0, both=True).translate((0.138, -EAR_CENTER_Y, 0.0))
    front_pin = _xzy_cylinder(PIN_R, PIN_SPAN, x=BAR2_LEN, y=0.0, z=0.0)
    front_head_left = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=BAR2_LEN, y=PIN_SPAN / 2.0 + WASHER_T / 2.0, z=0.0)
    front_head_right = _xzy_cylinder(PIN_HEAD_R, WASHER_T, x=BAR2_LEN, y=-PIN_SPAN / 2.0 - WASHER_T / 2.0, z=0.0)

    shape = (
        beam.union(rear_eye)
        .union(left_shoulder)
        .union(right_shoulder)
        .union(ear_left)
        .union(ear_right)
        .union(front_pin)
        .union(front_head_left)
        .union(front_head_right)
    )
    rear_bore = _xzy_cylinder(PIN_HOLE_R, PIN_SPAN + 0.010, x=0.0, y=0.0, z=0.0)
    return shape.cut(rear_bore)


def _make_bar_three_with_hook() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.066, PLATE_T, 0.017).translate((0.044, 0.0, 0.0))
    rear_eye = _xzy_cylinder(0.0125, PLATE_T, x=0.0, y=0.0, z=0.0)
    upright = cq.Workplane("XY").box(0.014, PLATE_T, 0.032).translate((0.092, 0.0, 0.002))
    top_nose = cq.Workplane("XY").box(0.022, PLATE_T, 0.008).translate((0.105, 0.0, 0.016))
    lower_shelf = cq.Workplane("XY").box(0.018, PLATE_T, 0.006).translate((0.097, 0.0, -0.013))
    front_lip = cq.Workplane("XY").box(0.006, PLATE_T, 0.018).translate((0.114, 0.0, 0.004))
    gusset = _extrude_xz_polygon([(0.070, -0.008), (0.086, -0.008), (0.094, 0.010), (0.074, 0.010)], PLATE_T)

    shape = (
        beam.union(rear_eye)
        .union(upright)
        .union(top_nose)
        .union(lower_shelf)
        .union(front_lip)
        .union(gusset)
    )
    rear_bore = _xzy_cylinder(PIN_HOLE_R, PIN_SPAN + 0.010, x=0.0, y=0.0, z=0.0)
    return shape.cut(rear_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_linkage_support")

    model.material("powder_black", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("zinc_steel", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("machined_pin", rgba=(0.82, 0.84, 0.86, 1.0))

    anchor_foot = model.part("anchor_foot")
    anchor_foot.visual(
        mesh_from_cadquery(_make_anchor_foot(), "anchor_foot"),
        material="powder_black",
        name="foot_body",
    )

    bar_one = model.part("bar_one")
    bar_one.visual(
        mesh_from_cadquery(_make_bar_one(), "bar_one"),
        material="zinc_steel",
        name="bar_one_body",
    )

    bar_two = model.part("bar_two")
    bar_two.visual(
        mesh_from_cadquery(_make_bar_two(), "bar_two"),
        material="zinc_steel",
        name="bar_two_body",
    )

    bar_three = model.part("bar_three")
    bar_three.visual(
        mesh_from_cadquery(_make_bar_three_with_hook(), "bar_three"),
        material="machined_pin",
        name="bar_three_body",
    )

    model.articulation(
        "foot_to_bar_one",
        ArticulationType.REVOLUTE,
        parent=anchor_foot,
        child=bar_one,
        origin=Origin(xyz=(0.0, 0.0, ROOT_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-1.30, upper=0.95),
    )
    model.articulation(
        "bar_one_to_bar_two",
        ArticulationType.REVOLUTE,
        parent=bar_one,
        child=bar_two,
        origin=Origin(xyz=(BAR1_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.4, lower=-2.15, upper=0.18),
    )
    model.articulation(
        "bar_two_to_bar_three",
        ArticulationType.REVOLUTE,
        parent=bar_two,
        child=bar_three,
        origin=Origin(xyz=(BAR2_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.8, lower=-1.75, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    anchor_foot = object_model.get_part("anchor_foot")
    bar_one = object_model.get_part("bar_one")
    bar_two = object_model.get_part("bar_two")
    bar_three = object_model.get_part("bar_three")

    foot_to_bar_one = object_model.get_articulation("foot_to_bar_one")
    bar_one_to_bar_two = object_model.get_articulation("bar_one_to_bar_two")
    bar_two_to_bar_three = object_model.get_articulation("bar_two_to_bar_three")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0010)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (anchor_foot, bar_one, bar_two, bar_three)),
        "Expected anchor foot and all three linkage bars to exist.",
    )
    ctx.check(
        "planar_joint_axes",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in (foot_to_bar_one, bar_one_to_bar_two, bar_two_to_bar_three)),
        "All three joints should revolve about the shared Y axis for one-plane folding.",
    )
    ctx.check(
        "staggered_bar_lengths",
        max(BAR1_LEN, BAR2_LEN, BAR3_LEN) - min(BAR1_LEN, BAR2_LEN, BAR3_LEN) > 0.05,
        "Bar lengths should be visibly staggered for a deliberate folded stack.",
    )
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.009)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=False, ignore_fixed=True)

    with ctx.pose(
        {
            foot_to_bar_one: 0.90,
            bar_one_to_bar_two: -2.10,
            bar_two_to_bar_three: 0.80,
        }
    ):
        ctx.expect_origin_distance(
            bar_three,
            anchor_foot,
            axes="x",
            max_dist=0.145,
            name="folded_hook_returns_near_anchor",
        )
        ctx.expect_origin_gap(
            bar_three,
            anchor_foot,
            axis="z",
            min_gap=0.060,
            name="folded_hook_stacks_above_foot",
        )

    with ctx.pose(
        {
            foot_to_bar_one: -0.30,
            bar_one_to_bar_two: -0.05,
            bar_two_to_bar_three: 0.35,
        }
    ):
        ctx.expect_origin_distance(
            bar_three,
            anchor_foot,
            axes="x",
            min_dist=0.245,
            name="deployed_hook_reaches_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
