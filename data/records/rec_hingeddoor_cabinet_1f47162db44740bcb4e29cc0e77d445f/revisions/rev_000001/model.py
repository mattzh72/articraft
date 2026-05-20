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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_front_display_cabinet")

    wood = model.material("wood", rgba=(0.52, 0.35, 0.21, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.33, 0.21, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.76, 0.88, 0.94, 0.28))

    cabinet_width = 0.92
    cabinet_depth = 0.38
    cabinet_height = 1.75

    plinth_height = 0.09
    top_height = 0.05
    side_thickness = 0.025
    back_thickness = 0.012

    door_thickness = 0.028
    door_bottom_gap = 0.008
    door_top_gap = 0.008
    meeting_gap = 0.004

    opening_width = cabinet_width - (2.0 * side_thickness)
    door_width = (opening_width - meeting_gap) / 2.0
    door_height = cabinet_height - plinth_height - top_height - door_bottom_gap - door_top_gap

    stile_width = 0.045
    rail_height = 0.060
    glass_capture = 0.006
    knob_inset_from_meeting_edge = 0.024
    knob_height = 0.84

    front_reveal = 0.004
    door_front_y = (cabinet_depth / 2.0) - front_reveal
    hinge_z = plinth_height + door_bottom_gap

    case = model.part("case")
    side_height = cabinet_height - plinth_height - top_height
    side_center_z = plinth_height + (side_height / 2.0)

    case.visual(
        Box((cabinet_width, cabinet_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=wood_dark,
        name="base_plinth",
    )
    case.visual(
        Box((cabinet_width, cabinet_depth, top_height)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - (top_height / 2.0))),
        material=wood_dark,
        name="top_cap",
    )
    case.visual(
        Box((side_thickness, cabinet_depth, side_height)),
        origin=Origin(
            xyz=(-(cabinet_width / 2.0) + (side_thickness / 2.0), 0.0, side_center_z)
        ),
        material=wood,
        name="left_side",
    )
    case.visual(
        Box((side_thickness, cabinet_depth, side_height)),
        origin=Origin(
            xyz=((cabinet_width / 2.0) - (side_thickness / 2.0), 0.0, side_center_z)
        ),
        material=wood,
        name="right_side",
    )
    case.visual(
        Box((cabinet_width - (2.0 * side_thickness), back_thickness, side_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth / 2.0) + (back_thickness / 2.0),
                side_center_z,
            )
        ),
        material=wood,
        name="back_panel",
    )

    def add_door(part_name: str, sign: float) -> None:
        door = model.part(part_name)

        door.visual(
            Box((stile_width, door_thickness, door_height)),
            origin=Origin(
                xyz=(sign * (stile_width / 2.0), -(door_thickness / 2.0), door_height / 2.0)
            ),
            material=wood,
            name="hinge_stile",
        )
        door.visual(
            Box((stile_width, door_thickness, door_height)),
            origin=Origin(
                xyz=(
                    sign * (door_width - (stile_width / 2.0)),
                    -(door_thickness / 2.0),
                    door_height / 2.0,
                )
            ),
            material=wood,
            name="meeting_stile",
        )
        door.visual(
            Box((door_width, door_thickness, rail_height)),
            origin=Origin(
                xyz=(sign * (door_width / 2.0), -(door_thickness / 2.0), rail_height / 2.0)
            ),
            material=wood,
            name="bottom_rail",
        )
        door.visual(
            Box((door_width, door_thickness, rail_height)),
            origin=Origin(
                xyz=(
                    sign * (door_width / 2.0),
                    -(door_thickness / 2.0),
                    door_height - (rail_height / 2.0),
                )
            ),
            material=wood,
            name="top_rail",
        )
        door.visual(
            Box(
                (
                    door_width - (2.0 * (stile_width - glass_capture)),
                    0.010,
                    door_height - (2.0 * (rail_height - glass_capture)),
                )
            ),
            origin=Origin(
                xyz=(sign * (door_width / 2.0), -(door_thickness / 2.0), door_height / 2.0)
            ),
            material=glass,
            name="glass_lite",
        )

    add_door("left_door", 1.0)
    add_door("right_door", -1.0)

    def add_knob(part_name: str) -> None:
        knob = model.part(part_name)

        knob.visual(
            Cylinder(radius=0.0105, length=0.0035),
            origin=Origin(xyz=(0.0, 0.00175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wood_dark,
            name="backplate",
        )
        knob.visual(
            Cylinder(radius=0.0038, length=0.012),
            origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=wood_dark,
            name="spindle",
        )
        knob.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.0, 0.018, 0.0)),
            material=wood,
            name="knob_head",
        )

    add_knob("left_knob")
    add_knob("right_knob")

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child="left_door",
        origin=Origin(
            xyz=(
                -(cabinet_width / 2.0) + side_thickness,
                door_front_y,
                hinge_z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.7,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child="right_door",
        origin=Origin(
            xyz=(
                (cabinet_width / 2.0) - side_thickness,
                door_front_y,
                hinge_z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.7,
        ),
    )
    model.articulation(
        "left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent="left_door",
        child="left_knob",
        origin=Origin(
            xyz=(
                door_width - knob_inset_from_meeting_edge,
                0.0,
                knob_height,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=10.0,
        ),
    )
    model.articulation(
        "right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent="right_door",
        child="right_knob",
        origin=Origin(
            xyz=(
                -(door_width - knob_inset_from_meeting_edge),
                0.0,
                knob_height,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=10.0,
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

    case = object_model.get_part("case")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    left_knob_spin = object_model.get_articulation("left_knob_spin")
    right_knob_spin = object_model.get_articulation("right_knob_spin")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.002,
            max_gap=0.008,
            name="closed doors keep a narrow meeting gap",
        )
        closed_case_aabb = ctx.part_world_aabb(case)
        closed_left_aabb = ctx.part_world_aabb(left_door)
        closed_right_aabb = ctx.part_world_aabb(right_door)

    ctx.check(
        "closed left door remains slightly recessed behind the case front",
        closed_case_aabb is not None
        and closed_left_aabb is not None
        and closed_left_aabb[1][1] <= closed_case_aabb[1][1] - 0.002,
        details=f"case={closed_case_aabb}, left={closed_left_aabb}",
    )
    ctx.check(
        "closed right door remains slightly recessed behind the case front",
        closed_case_aabb is not None
        and closed_right_aabb is not None
        and closed_right_aabb[1][1] <= closed_case_aabb[1][1] - 0.002,
        details=f"case={closed_case_aabb}, right={closed_right_aabb}",
    )

    with ctx.pose({left_hinge: 1.2, right_hinge: 1.2}):
        open_left_aabb = ctx.part_world_aabb(left_door)
        open_right_aabb = ctx.part_world_aabb(right_door)

    ctx.check(
        "left door swings outward from the case front",
        closed_left_aabb is not None
        and open_left_aabb is not None
        and open_left_aabb[1][1] > closed_left_aabb[1][1] + 0.10,
        details=f"closed={closed_left_aabb}, open={open_left_aabb}",
    )
    ctx.check(
        "right door swings outward from the case front",
        closed_right_aabb is not None
        and open_right_aabb is not None
        and open_right_aabb[1][1] > closed_right_aabb[1][1] + 0.10,
        details=f"closed={closed_right_aabb}, open={open_right_aabb}",
    )
    ctx.expect_contact(
        left_knob,
        left_door,
        elem_a="backplate",
        elem_b="meeting_stile",
        name="left knob backplate is mounted to the left door stile",
    )
    ctx.expect_contact(
        right_knob,
        right_door,
        elem_a="backplate",
        elem_b="meeting_stile",
        name="right knob backplate is mounted to the right door stile",
    )
    ctx.check(
        "left knob uses a local spin axis",
        left_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_knob_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={left_knob_spin.articulation_type}, axis={left_knob_spin.axis}",
    )
    ctx.check(
        "right knob uses a local spin axis",
        right_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_knob_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={right_knob_spin.articulation_type}, axis={right_knob_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
