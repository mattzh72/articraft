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
    model = ArticulatedObject(name="office_refrigerator")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.96, 0.96, 0.94, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    cabinet_width = 0.48
    cabinet_depth = 0.39
    cabinet_height = 0.60
    shell_thickness = 0.025
    stop_depth = 0.012
    stop_width = 0.012

    opening_width = cabinet_width - 2.0 * shell_thickness
    opening_height = cabinet_height - 2.0 * shell_thickness

    door_body_thickness = 0.036
    door_gasket_thickness = 0.004
    door_total_thickness = door_body_thickness + door_gasket_thickness
    door_clearance = 0.004
    door_width = opening_width - 2.0 * door_clearance
    door_height = opening_height - 2.0 * door_clearance

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((cabinet_depth, shell_thickness, cabinet_height)),
        origin=Origin(
            xyz=(0.0, -cabinet_width / 2.0 + shell_thickness / 2.0, cabinet_height / 2.0)
        ),
        material=shell_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((cabinet_depth, shell_thickness, cabinet_height)),
        origin=Origin(
            xyz=(0.0, cabinet_width / 2.0 - shell_thickness / 2.0, cabinet_height / 2.0)
        ),
        material=shell_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_depth, opening_width, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shell_thickness / 2.0)),
        material=shell_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((cabinet_depth, opening_width, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - shell_thickness / 2.0)),
        material=shell_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((shell_thickness, opening_width, opening_height)),
        origin=Origin(
            xyz=(
                -cabinet_depth / 2.0 + shell_thickness / 2.0,
                0.0,
                shell_thickness + opening_height / 2.0,
            )
        ),
        material=liner_white,
        name="back_panel",
    )

    cabinet.visual(
        Box((0.13, opening_width - 0.04, 0.10)),
        origin=Origin(
            xyz=(
                -cabinet_depth / 2.0 + shell_thickness + 0.065,
                0.0,
                shell_thickness + 0.05,
            )
        ),
        material=trim_gray,
        name="compressor_hump",
    )

    stop_center_x = cabinet_depth / 2.0 - door_total_thickness - stop_depth / 2.0
    cabinet.visual(
        Box((stop_depth, stop_width, opening_height)),
        origin=Origin(
            xyz=(stop_center_x, -opening_width / 2.0 + stop_width / 2.0, cabinet_height / 2.0)
        ),
        material=gasket_gray,
        name="left_stop",
    )
    cabinet.visual(
        Box((stop_depth, stop_width, opening_height)),
        origin=Origin(
            xyz=(stop_center_x, opening_width / 2.0 - stop_width / 2.0, cabinet_height / 2.0)
        ),
        material=gasket_gray,
        name="right_stop",
    )
    cabinet.visual(
        Box((stop_depth, opening_width - 2.0 * stop_width, stop_width)),
        origin=Origin(
            xyz=(stop_center_x, 0.0, shell_thickness + stop_width / 2.0)
        ),
        material=gasket_gray,
        name="bottom_stop",
    )
    cabinet.visual(
        Box((stop_depth, opening_width - 2.0 * stop_width, stop_width)),
        origin=Origin(
            xyz=(stop_center_x, 0.0, cabinet_height - shell_thickness - stop_width / 2.0)
        ),
        material=gasket_gray,
        name="top_stop",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_depth, cabinet_width, cabinet_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    door = model.part("door")

    door.visual(
        Box((door_body_thickness, door_width, door_height)),
        origin=Origin(xyz=(-door_body_thickness / 2.0, door_width / 2.0, 0.0)),
        material=shell_white,
        name="door_shell",
    )

    gasket_strip_width = 0.008
    door.visual(
        Box((door_gasket_thickness, door_width - 2.0 * stop_width, gasket_strip_width)),
        origin=Origin(
            xyz=(
                -door_body_thickness - door_gasket_thickness / 2.0,
                door_width / 2.0,
                door_height / 2.0 - gasket_strip_width / 2.0,
            )
        ),
        material=gasket_gray,
        name="top_gasket",
    )
    door.visual(
        Box((door_gasket_thickness, door_width - 2.0 * stop_width, gasket_strip_width)),
        origin=Origin(
            xyz=(
                -door_body_thickness - door_gasket_thickness / 2.0,
                door_width / 2.0,
                -door_height / 2.0 + gasket_strip_width / 2.0,
            )
        ),
        material=gasket_gray,
        name="bottom_gasket",
    )
    door.visual(
        Box((door_gasket_thickness, gasket_strip_width, door_height - 2.0 * stop_width)),
        origin=Origin(
            xyz=(
                -door_body_thickness - door_gasket_thickness / 2.0,
                gasket_strip_width / 2.0,
                0.0,
            )
        ),
        material=gasket_gray,
        name="hinge_gasket",
    )
    door.visual(
        Box((door_gasket_thickness, gasket_strip_width, door_height - 2.0 * stop_width)),
        origin=Origin(
            xyz=(
                -door_body_thickness - door_gasket_thickness / 2.0,
                door_width - gasket_strip_width / 2.0,
                0.0,
            )
        ),
        material=gasket_gray,
        name="latch_gasket",
    )

    latch_pad_width = 0.050
    latch_pad_height = 0.060
    latch_pad_thickness = 0.006
    latch_pivot_y = door_width - 0.050
    latch_pivot_z = door_height / 2.0 - 0.055
    door.visual(
        Box((latch_pad_thickness, latch_pad_width, latch_pad_height)),
        origin=Origin(
            xyz=(
                latch_pad_thickness / 2.0,
                latch_pivot_y - latch_pad_width / 2.0,
                latch_pivot_z,
            )
        ),
        material=trim_gray,
        name="latch_pad",
    )
    door.visual(
        Box((0.004, 0.065, 0.012)),
        origin=Origin(
            xyz=(0.002, door_width / 2.0, door_height / 2.0 - 0.014)
        ),
        material=trim_gray,
        name="top_trim",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_total_thickness, door_width, door_height)),
        mass=5.0,
        origin=Origin(xyz=(-door_total_thickness / 2.0, door_width / 2.0, 0.0)),
    )

    latch = model.part("latch_lever")

    pivot_radius = 0.008
    pivot_length = 0.010
    lever_length = 0.075
    latch.visual(
        Cylinder(radius=pivot_radius, length=pivot_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_metal,
        name="latch_pivot",
    )
    latch.visual(
        Box((0.008, lever_length, 0.016)),
        origin=Origin(xyz=(0.0, -lever_length / 2.0, 0.0)),
        material=latch_metal,
        name="latch_arm",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(0.0, -lever_length, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_metal,
        name="latch_grip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.014, lever_length + 0.014, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -lever_length / 2.0, 0.0)),
    )

    door_hinge_origin = Origin(
        xyz=(
            cabinet_depth / 2.0,
            -opening_width / 2.0 + door_clearance,
            shell_thickness + opening_height / 2.0,
        )
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=door_hinge_origin,
        # Closed door spans local +Y from the left hinge line.
        # -Z makes positive rotation swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(
            xyz=(
                latch_pad_thickness + pivot_length / 2.0,
                latch_pivot_y,
                latch_pivot_z,
            )
        ),
        # Closed lever extends along local -Y; -X makes positive motion lift it upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(75.0),
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
    door = object_model.get_part("door")
    latch = object_model.get_part("latch_lever")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.expect_contact(
        door,
        cabinet,
        contact_tol=0.001,
        name="door gasket seats against cabinet stop frame",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        min_overlap=0.40,
        name="door covers the refrigerator opening",
    )
    ctx.expect_contact(
        latch,
        door,
        contact_tol=0.001,
        elem_a="latch_pivot",
        elem_b="latch_pad",
        name="latch lever is mounted on the door pad",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(95.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward with positive hinge motion",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: math.radians(65.0)}):
        raised_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch lever lifts upward with positive motion",
        closed_latch_aabb is not None
        and raised_latch_aabb is not None
        and raised_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.035,
        details=f"closed={closed_latch_aabb}, raised={raised_latch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
