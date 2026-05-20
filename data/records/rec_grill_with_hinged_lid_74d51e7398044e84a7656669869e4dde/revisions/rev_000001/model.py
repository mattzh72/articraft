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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_top_griddle")

    body_black = model.material("body_black", rgba=(0.14, 0.14, 0.15, 1.0))
    cook_steel = model.material("cook_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    cover_steel = model.material("cover_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.52, 0.54, 0.57, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.86, 0.87, 0.89, 1.0))

    stand_body = model.part("stand_body")
    stand_body.visual(
        Box((0.78, 0.50, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=body_black,
        name="upper_body",
    )
    stand_body.visual(
        Box((0.78, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, 0.2975, 0.745)),
        material=body_black,
        name="control_panel",
    )
    stand_body.visual(
        Box((0.06, 0.46, 0.735)),
        origin=Origin(xyz=(-0.36, 0.0, 0.3675)),
        material=body_black,
        name="left_leg",
    )
    stand_body.visual(
        Box((0.06, 0.46, 0.735)),
        origin=Origin(xyz=(0.36, 0.0, 0.3675)),
        material=body_black,
        name="right_leg",
    )
    stand_body.visual(
        Box((0.72, 0.40, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=body_black,
        name="lower_shelf",
    )
    stand_body.visual(
        Box((0.72, 0.02, 0.505)),
        origin=Origin(xyz=(0.0, -0.21, 0.4875)),
        material=body_black,
        name="rear_panel",
    )
    stand_body.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, -0.18, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="axle_bar",
    )
    stand_body.visual(
        Cylinder(radius=0.015, length=0.082),
        origin=Origin(xyz=(-0.401, -0.18, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="left_axle_stub",
    )
    stand_body.visual(
        Cylinder(radius=0.015, length=0.082),
        origin=Origin(xyz=(0.401, -0.18, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="right_axle_stub",
    )
    stand_body.visual(
        Box((0.72, 0.46, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.919)),
        material=cook_steel,
        name="cook_plate",
    )
    stand_body.visual(
        Box((0.72, 0.02, 0.07)),
        origin=Origin(xyz=(0.0, -0.21, 0.963)),
        material=cook_steel,
        name="rear_splash",
    )
    stand_body.visual(
        Box((0.72, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, -0.245, 0.885)),
        material=body_black,
        name="rear_hinge_mount",
    )
    stand_body.inertial = Inertial.from_geometry(
        Box((0.96, 0.62, 0.94)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cover_steel,
        name="hinge_roll",
    )
    cover.visual(
        Box((0.76, 0.40, 0.02)),
        origin=Origin(xyz=(0.0, 0.20, 0.13)),
        material=cover_steel,
        name="cover_top",
    )
    cover.visual(
        Box((0.76, 0.025, 0.122)),
        origin=Origin(xyz=(0.0, 0.0125, 0.061)),
        material=cover_steel,
        name="cover_back_panel",
    )
    cover.visual(
        Box((0.76, 0.02, 0.122)),
        origin=Origin(xyz=(0.0, 0.39, 0.061)),
        material=cover_steel,
        name="cover_front_panel",
    )
    cover.visual(
        Box((0.02, 0.365, 0.122)),
        origin=Origin(xyz=(-0.37, 0.1975, 0.061)),
        material=cover_steel,
        name="left_cover_side",
    )
    cover.visual(
        Box((0.02, 0.365, 0.122)),
        origin=Origin(xyz=(0.37, 0.1975, 0.061)),
        material=cover_steel,
        name="right_cover_side",
    )
    cover.visual(
        Box((0.03, 0.025, 0.04)),
        origin=Origin(xyz=(-0.20, 0.4025, 0.07)),
        material=cover_steel,
        name="left_handle_post",
    )
    cover.visual(
        Box((0.03, 0.025, 0.04)),
        origin=Origin(xyz=(0.20, 0.4025, 0.07)),
        material=cover_steel,
        name="right_handle_post",
    )
    cover.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(0.0, 0.425, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cover_steel,
        name="handle_bar",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.78, 0.44, 0.16)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.20, 0.08)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="hub",
    )
    left_wheel.visual(
        Box((0.008, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=hub_grey,
        name="valve_stem",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Box((0.04, 0.22, 0.22)),
        mass=2.2,
        origin=Origin(),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="hub",
    )
    right_wheel.visual(
        Box((0.008, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=hub_grey,
        name="valve_stem",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Box((0.04, 0.22, 0.22)),
        mass=2.2,
        origin=Origin(),
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=0.035, length=0.03),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    left_knob.visual(
        Box((0.008, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, 0.022)),
        material=knob_mark,
        name="pointer",
    )
    left_knob.inertial = Inertial.from_geometry(
        Box((0.07, 0.03, 0.07)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=0.035, length=0.03),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    right_knob.visual(
        Box((0.008, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, 0.022)),
        material=knob_mark,
        name="pointer",
    )
    right_knob.inertial = Inertial.from_geometry(
        Box((0.07, 0.03, 0.07)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )

    model.articulation(
        "stand_to_cover",
        ArticulationType.REVOLUTE,
        parent=stand_body,
        child=cover,
        origin=Origin(xyz=(0.0, -0.245, 0.93)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "stand_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=stand_body,
        child=left_wheel,
        origin=Origin(xyz=(-0.462, -0.18, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=12.0),
    )
    model.articulation(
        "stand_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=stand_body,
        child=right_wheel,
        origin=Origin(xyz=(0.462, -0.18, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=12.0),
    )
    model.articulation(
        "stand_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=stand_body,
        child=left_knob,
        origin=Origin(xyz=(-0.12, 0.3525, 0.745)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "stand_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=stand_body,
        child=right_knob,
        origin=Origin(xyz=(0.12, 0.3525, 0.745)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
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

    stand_body = object_model.get_part("stand_body")
    cover = object_model.get_part("cover")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")

    cover_hinge = object_model.get_articulation("stand_to_cover")
    left_wheel_joint = object_model.get_articulation("stand_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("stand_to_right_wheel")
    left_knob_joint = object_model.get_articulation("stand_to_left_knob")
    right_knob_joint = object_model.get_articulation("stand_to_right_knob")

    def _type_name(joint) -> str:
        joint_type = joint.articulation_type
        return getattr(joint_type, "name", str(joint_type).split(".")[-1])

    def _center_from_aabb(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.check(
        "cover hinge is rear x-axis revolute",
        _type_name(cover_hinge) == "REVOLUTE" and tuple(round(value, 3) for value in cover_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={_type_name(cover_hinge)}, axis={cover_hinge.axis}",
    )
    ctx.check(
        "wheel joints are continuous axle spins",
        _type_name(left_wheel_joint) == "CONTINUOUS"
        and _type_name(right_wheel_joint) == "CONTINUOUS"
        and tuple(round(value, 3) for value in left_wheel_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(round(value, 3) for value in right_wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left=({_type_name(left_wheel_joint)}, {left_wheel_joint.axis}), "
            f"right=({_type_name(right_wheel_joint)}, {right_wheel_joint.axis})"
        ),
    )
    ctx.check(
        "control knobs rotate on front to back shafts",
        _type_name(left_knob_joint) == "CONTINUOUS"
        and _type_name(right_knob_joint) == "CONTINUOUS"
        and tuple(round(value, 3) for value in left_knob_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(value, 3) for value in right_knob_joint.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left=({_type_name(left_knob_joint)}, {left_knob_joint.axis}), "
            f"right=({_type_name(right_knob_joint)}, {right_knob_joint.axis})"
        ),
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            stand_body,
            axes="xy",
            elem_a="cover_top",
            elem_b="cook_plate",
            min_overlap=0.38,
            name="cover spans the cooking plate when closed",
        )
        ctx.expect_gap(
            cover,
            stand_body,
            axis="z",
            positive_elem="cover_front_panel",
            negative_elem="cook_plate",
            min_gap=0.0,
            max_gap=0.008,
            name="closed cover sits just above the cook plate",
        )

    closed_handle = _center_from_aabb(cover, "handle_bar")
    with ctx.pose({cover_hinge: 1.10}):
        open_handle = _center_from_aabb(cover, "handle_bar")
    ctx.check(
        "cover lifts upward on its rear hinge",
        closed_handle is not None
        and open_handle is not None
        and open_handle[2] > closed_handle[2] + 0.20
        and open_handle[1] < closed_handle[1] - 0.10,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    ctx.expect_contact(
        left_wheel,
        stand_body,
        elem_a="hub",
        elem_b="left_axle_stub",
        name="left wheel is mounted on the left axle stub",
    )
    ctx.expect_contact(
        right_wheel,
        stand_body,
        elem_a="hub",
        elem_b="right_axle_stub",
        name="right wheel is mounted on the right axle stub",
    )

    left_stem_rest = _center_from_aabb(left_wheel, "valve_stem")
    with ctx.pose({left_wheel_joint: math.pi / 2.0}):
        left_stem_spun = _center_from_aabb(left_wheel, "valve_stem")
    ctx.check(
        "left wheel rotation moves a rim feature around the axle",
        left_stem_rest is not None
        and left_stem_spun is not None
        and left_stem_spun[2] > left_stem_rest[2] + 0.08,
        details=f"rest={left_stem_rest}, spun={left_stem_spun}",
    )

    ctx.expect_contact(
        left_knob,
        stand_body,
        elem_a="knob_body",
        elem_b="control_panel",
        name="left knob is mounted on the control panel",
    )
    ctx.expect_contact(
        right_knob,
        stand_body,
        elem_a="knob_body",
        elem_b="control_panel",
        name="right knob is mounted on the control panel",
    )

    left_pointer_rest = _center_from_aabb(left_knob, "pointer")
    with ctx.pose({left_knob_joint: math.pi / 2.0}):
        left_pointer_turned = _center_from_aabb(left_knob, "pointer")
    ctx.check(
        "left knob rotation turns its indicator around the shaft",
        left_pointer_rest is not None
        and left_pointer_turned is not None
        and left_pointer_turned[0] > left_pointer_rest[0] + 0.015,
        details=f"rest={left_pointer_rest}, turned={left_pointer_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
