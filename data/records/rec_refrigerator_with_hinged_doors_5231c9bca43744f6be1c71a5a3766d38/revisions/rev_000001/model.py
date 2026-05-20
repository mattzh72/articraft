from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beverage_refrigerator")

    exterior_black = model.material("exterior_black", rgba=(0.10, 0.11, 0.12, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.75, 0.77, 0.79, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.58, 0.60, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.62, 0.76, 0.88, 0.28))
    shelf_glass = model.material("shelf_glass", rgba=(0.70, 0.82, 0.92, 0.20))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.15, 0.10, 1.0))

    cabinet_width = 0.46
    cabinet_depth = 0.52
    cabinet_height = 0.86
    wall_thickness = 0.028
    back_thickness = 0.018
    base_thickness = 0.030
    top_thickness = 0.030

    door_thickness = 0.035
    door_height = 0.77
    door_bottom = 0.050
    door_frame = 0.045
    glass_overlap = 0.012

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2 + wall_thickness / 2, 0.0, cabinet_height / 2)),
        material=exterior_black,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2 - wall_thickness / 2, 0.0, cabinet_height / 2)),
        material=exterior_black,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2)),
        material=exterior_black,
        name="base_plinth",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - top_thickness / 2)),
        material=exterior_black,
        name="top_cap",
    )
    cabinet.visual(
        Box((cabinet_width - 2 * wall_thickness, back_thickness, cabinet_height)),
        origin=Origin(
            xyz=(0.0, cabinet_depth / 2 - back_thickness / 2, cabinet_height / 2)
        ),
        material=interior_gray,
        name="back_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2 * wall_thickness, 0.060, top_thickness)),
        origin=Origin(
            xyz=(0.0, -cabinet_depth / 2 + 0.030, cabinet_height - top_thickness / 2)
        ),
        material=interior_gray,
        name="inner_top_frame",
    )

    shelf_width = cabinet_width - 2 * wall_thickness + 0.004
    shelf_depth = cabinet_depth - back_thickness - 0.110
    shelf_center_y = 0.025
    shelf_thickness = 0.006
    for index, shelf_z in enumerate((0.29, 0.53), start=1):
        cabinet.visual(
            Box((shelf_width, shelf_depth, shelf_thickness)),
            origin=Origin(xyz=(0.0, shelf_center_y, shelf_z)),
            material=shelf_glass,
            name=f"shelf_{index}",
        )
        cabinet.visual(
            Box((shelf_width, 0.012, 0.018)),
            origin=Origin(
                xyz=(0.0, shelf_center_y - shelf_depth / 2 + 0.006, shelf_z + 0.006)
            ),
            material=trim_gray,
            name=f"shelf_trim_{index}",
        )

    door = model.part("door")
    door.visual(
        Box((door_frame, door_thickness, door_height)),
        origin=Origin(xyz=(door_frame / 2, 0.0, door_height / 2)),
        material=exterior_black,
        name="hinge_stile",
    )
    door.visual(
        Box((door_frame, door_thickness, door_height)),
        origin=Origin(xyz=(cabinet_width - door_frame / 2, 0.0, door_height / 2)),
        material=exterior_black,
        name="latch_stile",
    )
    door.visual(
        Box((cabinet_width, door_thickness, door_frame)),
        origin=Origin(xyz=(cabinet_width / 2, 0.0, door_frame / 2)),
        material=exterior_black,
        name="bottom_rail",
    )
    door.visual(
        Box((cabinet_width, door_thickness, door_frame)),
        origin=Origin(xyz=(cabinet_width / 2, 0.0, door_height - door_frame / 2)),
        material=exterior_black,
        name="top_rail",
    )
    door.visual(
        Box(
            (
                cabinet_width - 2 * (door_frame - glass_overlap),
                0.006,
                door_height - 2 * (door_frame - glass_overlap),
            )
        ),
        origin=Origin(
            xyz=(
                cabinet_width / 2,
                -0.006,
                door_height / 2,
            )
        ),
        material=glass_tint,
        name="glass_panel",
    )
    door.visual(
        Box((0.018, 0.026, 0.44)),
        origin=Origin(
            xyz=(cabinet_width - 0.030, -door_thickness / 2 - 0.013, door_height / 2)
        ),
        material=trim_gray,
        name="handle_rail",
    )

    knob = model.part("thermostat_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=trim_gray,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.009, 0.0, -0.014)),
        material=accent_red,
        name="knob_pointer",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(
            xyz=(-cabinet_width / 2, -cabinet_depth / 2 - door_thickness / 2, door_bottom)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "cabinet_to_thermostat_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(
            xyz=(
                cabinet_width / 2 - wall_thickness - 0.055,
                -cabinet_depth / 2 + 0.055,
                cabinet_height - top_thickness,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.4,
            upper=1.4,
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
    knob = object_model.get_part("thermostat_knob")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    knob_joint = object_model.get_articulation("cabinet_to_thermostat_knob")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) / 2.0 for index in range(3))

    ctx.check(
        "door hinge axis is vertical",
        door_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "thermostat knob axis is vertical",
        knob_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={knob_joint.axis}",
    )

    with ctx.pose({door_hinge: 0.0, knob_joint: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="door closes flush to the cabinet front",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="xz",
            min_overlap=0.20,
            name="door covers the cabinet opening footprint",
        )
        ctx.expect_within(
            knob,
            cabinet,
            axes="xy",
            margin=0.0,
            name="thermostat knob sits inside the cabinet envelope",
        )
        closed_door_aabb = ctx.part_world_aabb(door)
        closed_pointer_aabb = ctx.part_element_world_aabb(knob, elem="knob_pointer")

    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)

    closed_door_center = aabb_center(closed_door_aabb)
    open_door_center = aabb_center(open_door_aabb)
    ctx.check(
        "door swings outward when opened",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10
        and open_door_center is not None
        and closed_door_center is not None
        and open_door_center[0] < closed_door_center[0] - 0.05,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({knob_joint: 0.90}):
        turned_pointer_aabb = ctx.part_element_world_aabb(knob, elem="knob_pointer")

    closed_pointer_center = aabb_center(closed_pointer_aabb)
    turned_pointer_center = aabb_center(turned_pointer_aabb)
    ctx.check(
        "thermostat knob pointer changes direction when rotated",
        closed_pointer_center is not None
        and turned_pointer_center is not None
        and (
            abs(turned_pointer_center[0] - closed_pointer_center[0]) > 0.003
            or abs(turned_pointer_center[1] - closed_pointer_center[1]) > 0.003
        )
        and abs(turned_pointer_center[2] - closed_pointer_center[2]) < 0.003,
        details=f"closed={closed_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
