from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_jobsite_speaker")

    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.72, 0.08, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    grille_black = model.material("grille_black", rgba=(0.05, 0.05, 0.05, 1.0))
    cone_black = model.material("cone_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_orange = model.material("knob_orange", rgba=(0.90, 0.45, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.32, 0.38)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )

    # Side loops and protective cage.
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_pos = side_sign * 0.225
        frame.visual(
            Box((0.022, 0.022, 0.280)),
            origin=Origin(xyz=(x_pos, -0.130, 0.180)),
            material=safety_yellow,
            name=f"{side_name}_front_upright",
        )
        frame.visual(
            Box((0.022, 0.022, 0.280)),
            origin=Origin(xyz=(x_pos, 0.130, 0.180)),
            material=safety_yellow,
            name=f"{side_name}_rear_upright",
        )
        frame.visual(
            Box((0.022, 0.282, 0.022)),
            origin=Origin(xyz=(x_pos, 0.0, 0.315)),
            material=safety_yellow,
            name=f"{side_name}_top_rail",
        )
        frame.visual(
            Box((0.022, 0.300, 0.040)),
            origin=Origin(xyz=(x_pos, 0.0, 0.020)),
            material=safety_yellow,
            name=f"{side_name}_bottom_skid",
        )
        frame.visual(
            Box((0.028, 0.030, 0.040)),
            origin=Origin(xyz=(x_pos, -0.130, 0.020)),
            material=dark_rubber,
            name=f"{side_name}_front_foot",
        )
        frame.visual(
            Box((0.028, 0.030, 0.040)),
            origin=Origin(xyz=(x_pos, 0.130, 0.020)),
            material=dark_rubber,
            name=f"{side_name}_rear_foot",
        )

    frame.visual(
        Box((0.450, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, -0.130, 0.315)),
        material=safety_yellow,
        name="top_frame_front",
    )
    frame.visual(
        Box((0.450, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.130, 0.315)),
        material=safety_yellow,
        name="top_frame_rear",
    )
    frame.visual(
        Box((0.450, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, -0.130, 0.020)),
        material=safety_yellow,
        name="bottom_frame_front",
    )
    frame.visual(
        Box((0.450, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.130, 0.020)),
        material=safety_yellow,
        name="bottom_frame_rear",
    )
    frame.visual(
        Box((0.474, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.060, 0.034)),
        material=steel,
        name="front_cradle_bar",
    )
    frame.visual(
        Box((0.474, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.034)),
        material=steel,
        name="rear_cradle_bar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(-0.237, 0.0, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="left_handle_boss",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.237, 0.0, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="right_handle_boss",
    )

    speaker_body = model.part("speaker_body")
    speaker_body.inertial = Inertial.from_geometry(
        Box((0.38, 0.17, 0.27)),
        mass=6.6,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    body_w = 0.38
    body_d = 0.17
    body_h = 0.27
    wall = 0.012

    speaker_body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=charcoal,
        name="body_bottom_panel",
    )
    speaker_body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall * 0.5)),
        material=charcoal,
        name="body_top_panel",
    )
    speaker_body.visual(
        Box((wall, body_d, body_h - 2.0 * wall)),
        origin=Origin(xyz=(-body_w * 0.5 + wall * 0.5, 0.0, body_h * 0.5)),
        material=charcoal,
        name="body_left_wall",
    )
    speaker_body.visual(
        Box((wall, body_d, body_h - 2.0 * wall)),
        origin=Origin(xyz=(body_w * 0.5 - wall * 0.5, 0.0, body_h * 0.5)),
        material=charcoal,
        name="body_right_wall",
    )
    speaker_body.visual(
        Box((body_w - 2.0 * wall, wall, body_h - 2.0 * wall)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall * 0.5, body_h * 0.5)),
        material=charcoal,
        name="body_back_wall",
    )
    speaker_body.visual(
        Box((body_w, wall, 0.024)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall * 0.5, 0.258)),
        material=charcoal,
        name="front_top_strip",
    )
    speaker_body.visual(
        Box((body_w, wall, 0.024)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall * 0.5, 0.012)),
        material=charcoal,
        name="front_bottom_strip",
    )
    speaker_body.visual(
        Box((0.030, wall, 0.222)),
        origin=Origin(xyz=(-0.175, -body_d * 0.5 + wall * 0.5, 0.135)),
        material=charcoal,
        name="front_left_strip",
    )
    speaker_body.visual(
        Box((0.030, wall, 0.222)),
        origin=Origin(xyz=(0.175, -body_d * 0.5 + wall * 0.5, 0.135)),
        material=charcoal,
        name="front_right_strip",
    )
    speaker_body.visual(
        Box((0.356, 0.008, 0.246)),
        origin=Origin(xyz=(0.0, -0.058, 0.135)),
        material=charcoal,
        name="inner_baffle",
    )
    speaker_body.visual(
        Box((0.320, 0.004, 0.222)),
        origin=Origin(xyz=(0.0, -0.075, 0.135)),
        material=grille_black,
        name="front_grille",
    )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_pos = side_sign * 0.090
        speaker_body.visual(
            Cylinder(radius=0.068, length=0.010),
            origin=Origin(xyz=(x_pos, -0.053, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grille_black,
            name=f"{side_name}_woofer_ring",
        )
        speaker_body.visual(
            Cylinder(radius=0.056, length=0.026),
            origin=Origin(xyz=(x_pos, -0.041, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
            material=cone_black,
            name=f"{side_name}_woofer_cone",
        )
        speaker_body.visual(
            Cylinder(radius=0.012, length=0.028),
            origin=Origin(xyz=(x_pos, -0.040, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{side_name}_dust_cap",
        )

    speaker_body.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, -0.053, 0.196), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grille_black,
        name="tweeter_housing",
    )
    speaker_body.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, -0.043, 0.196), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cone_black,
        name="tweeter_dome",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.50, 0.05, 0.12)),
        mass=0.7,
        origin=Origin(xyz=(0.249, 0.0, 0.040)),
    )
    handle_path = wire_from_points(
        [
            (-0.012, 0.0, 0.000),
            (-0.024, 0.0, 0.032),
            (0.060, 0.0, 0.075),
            (0.438, 0.0, 0.075),
            (0.522, 0.0, 0.032),
            (0.510, 0.0, 0.000),
        ],
        radius=0.012,
        radial_segments=18,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=10,
    )
    handle.visual(
        _save_mesh("speaker_handle_tube", handle_path),
        material=safety_yellow,
        name="handle_tube",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="handle_left_pivot",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.504, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="handle_right_pivot",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.249, 0.0, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_rubber,
        name="handle_grip",
    )

    volume_knob = model.part("volume_knob")
    volume_knob.inertial = Inertial.from_geometry(
        Box((0.045, 0.040, 0.045)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )
    volume_knob.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_shaft",
    )
    volume_knob.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_orange,
        name="knob_body",
    )
    volume_knob.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="knob_face",
    )
    volume_knob.visual(
        Box((0.006, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.033, 0.012)),
        material=steel,
        name="knob_pointer",
    )

    model.articulation(
        "frame_to_body",
        ArticulationType.FIXED,
        parent=frame,
        child=speaker_body,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    model.articulation(
        "frame_to_handle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(-0.249, 0.0, 0.305)),
        # The carry handle rises along local +Z from the pivot line.
        # Rotating about +X makes negative q fold it rearward.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-1.15,
            upper=0.55,
        ),
    )
    model.articulation(
        "body_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=speaker_body,
        child=volume_knob,
        origin=Origin(xyz=(0.173, -0.085, 0.226)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    speaker_body = object_model.get_part("speaker_body")
    handle = object_model.get_part("handle")
    volume_knob = object_model.get_part("volume_knob")

    handle_joint = object_model.get_articulation("frame_to_handle")
    knob_joint = object_model.get_articulation("body_to_volume_knob")

    ctx.expect_contact(
        speaker_body,
        frame,
        name="speaker body is mounted into the protective frame",
    )
    ctx.expect_within(
        speaker_body,
        frame,
        axes="xy",
        margin=0.03,
        name="speaker body stays inside the frame footprint",
    )
    ctx.expect_contact(
        handle,
        frame,
        elem_a="handle_left_pivot",
        elem_b="left_handle_boss",
        name="left side of the folding handle is carried by a pivot boss",
    )
    ctx.expect_contact(
        handle,
        frame,
        elem_a="handle_right_pivot",
        elem_b="right_handle_boss",
        name="right side of the folding handle is carried by a pivot boss",
    )
    ctx.expect_contact(
        volume_knob,
        speaker_body,
        elem_a="knob_shaft",
        elem_b="front_right_strip",
        name="volume knob shaft is mounted on the upper-right front strip",
    )

    ctx.check(
        "handle pivots on a transverse axis",
        handle_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "volume knob rotates on a front-facing shaft",
        abs(knob_joint.axis[0]) < 1e-9
        and abs(abs(knob_joint.axis[1]) - 1.0) < 1e-9
        and abs(knob_joint.axis[2]) < 1e-9,
        details=f"axis={knob_joint.axis}",
    )
    ctx.check(
        "volume knob sits in the upper right corner",
        knob_joint.origin.xyz[0] > 0.15 and knob_joint.origin.xyz[2] > 0.21,
        details=f"origin={knob_joint.origin.xyz}",
    )

    ctx.expect_gap(
        handle,
        frame,
        axis="z",
        min_gap=0.025,
        positive_elem="handle_grip",
        negative_elem="top_frame_front",
        name="raised handle grip sits above the top frame rail",
    )

    rest_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))
    with ctx.pose({handle_joint: -0.95}):
        folded_handle_center = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_grip"))
        ctx.expect_gap(
            handle,
            speaker_body,
            axis="z",
            min_gap=0.015,
            positive_elem="handle_grip",
            negative_elem="body_top_panel",
            name="folded handle still clears the top of the speaker body",
        )

    ctx.check(
        "handle folds rearward from the carry position",
        rest_handle_center is not None
        and folded_handle_center is not None
        and folded_handle_center[1] > rest_handle_center[1] + 0.03
        and folded_handle_center[2] < rest_handle_center[2] - 0.03,
        details=f"rest={rest_handle_center}, folded={folded_handle_center}",
    )

    rest_pointer_center = _aabb_center(ctx.part_element_world_aabb(volume_knob, elem="knob_pointer"))
    with ctx.pose({knob_joint: pi / 2.0}):
        turned_pointer_center = _aabb_center(ctx.part_element_world_aabb(volume_knob, elem="knob_pointer"))
    ctx.check(
        "volume pointer moves when the knob turns",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[0] - rest_pointer_center[0]) > 0.007
        and abs(turned_pointer_center[2] - rest_pointer_center[2]) > 0.007,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
