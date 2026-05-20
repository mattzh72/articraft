from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_wagon_cart")

    red = Material("powder_coated_red", rgba=(0.78, 0.05, 0.035, 1.0))
    dark = Material("black_oxide_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    steel = Material("brushed_steel", rgba=(0.68, 0.68, 0.62, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    wood = Material("sealed_wood_planks", rgba=(0.55, 0.34, 0.16, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.117,
            0.064,
            rim=WheelRim(
                inner_radius=0.077,
                flange_height=0.008,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.029,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.040,
                    hole_diameter=0.004,
                ),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "galvanized_utility_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.150,
            0.078,
            inner_radius=0.116,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.008, count=22, land_ratio=0.55),
            grooves=(
                TireGroove(center_offset=-0.018, width=0.005, depth=0.003),
                TireGroove(center_offset=0.018, width=0.005, depth=0.003),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "knobby_wagon_tire",
    )

    bed = model.part("cargo_bed")
    # A shallow open steel tub with a real floor and four raised walls.
    bed.visual(Box((1.05, 0.58, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.330)), material=wood, name="wood_floor")
    bed.visual(Box((1.07, 0.035, 0.270)), origin=Origin(xyz=(0.0, 0.307, 0.482)), material=red, name="side_wall_0")
    bed.visual(Box((1.07, 0.035, 0.270)), origin=Origin(xyz=(0.0, -0.307, 0.482)), material=red, name="side_wall_1")
    bed.visual(Box((0.035, 0.615, 0.270)), origin=Origin(xyz=(0.535, 0.0, 0.482)), material=red, name="front_wall")
    bed.visual(Box((0.035, 0.615, 0.270)), origin=Origin(xyz=(-0.535, 0.0, 0.482)), material=red, name="rear_wall")

    # Rolled lip, corner posts, and pressed ribs make the bed read as a sturdy cart tub.
    for y, suffix in ((0.327, "0"), (-0.327, "1")):
        bed.visual(
            Cylinder(radius=0.014, length=1.075),
            origin=Origin(xyz=(0.0, y, 0.624), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=red,
            name=f"top_side_rail_{suffix}",
        )
    for x, suffix in ((0.552, "front"), (-0.552, "rear")):
        bed.visual(
            Cylinder(radius=0.014, length=0.625),
            origin=Origin(xyz=(x, 0.0, 0.624), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=red,
            name=f"{suffix}_top_rail",
        )
    for x, sx in ((0.535, "front"), (-0.535, "rear")):
        for y, sy in ((0.307, "0"), (-0.307, "1")):
            bed.visual(
                Box((0.045, 0.045, 0.305)),
                origin=Origin(xyz=(x, y, 0.482)),
                material=red,
                name=f"{sx}_corner_post_{sy}",
            )
    for x, suffix in ((-0.30, "rear"), (0.0, "center"), (0.30, "front")):
        bed.visual(Box((0.035, 0.040, 0.230)), origin=Origin(xyz=(x, 0.330, 0.485)), material=red, name=f"side_rib_0_{suffix}")
        bed.visual(Box((0.035, 0.040, 0.230)), origin=Origin(xyz=(x, -0.330, 0.485)), material=red, name=f"side_rib_1_{suffix}")
    for y, suffix in ((-0.190, "0"), (0.0, "1"), (0.190, "2")):
        bed.visual(Box((1.01, 0.018, 0.010)), origin=Origin(xyz=(0.0, y, 0.352)), material=dark, name=f"floor_plank_gap_{suffix}")

    # Exposed undercarriage: rails, crossmembers, rear fixed axle and a kingpin seat.
    bed.visual(Box((0.92, 0.040, 0.050)), origin=Origin(xyz=(0.0, 0.180, 0.2875)), material=dark, name="frame_rail_0")
    bed.visual(Box((0.92, 0.040, 0.050)), origin=Origin(xyz=(0.0, -0.180, 0.2875)), material=dark, name="frame_rail_1")
    for x, suffix in ((-0.415, "rear"), (0.0, "center")):
        bed.visual(Box((0.045, 0.440, 0.040)), origin=Origin(xyz=(x, 0.0, 0.2825)), material=dark, name=f"{suffix}_crossmember")
    bed.visual(Box((0.045, 0.130, 0.040)), origin=Origin(xyz=(0.360, 0.155, 0.2825)), material=dark, name="front_crossmember_0")
    bed.visual(Box((0.045, 0.130, 0.040)), origin=Origin(xyz=(0.360, -0.155, 0.2825)), material=dark, name="front_crossmember_1")
    bed.visual(
        Cylinder(radius=0.015, length=0.635),
        origin=Origin(xyz=(-0.385, 0.0, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    for y, suffix in ((0.180, "0"), (-0.180, "1")):
        bed.visual(Box((0.055, 0.035, 0.110)), origin=Origin(xyz=(-0.385, y, 0.210)), material=dark, name=f"rear_axle_hanger_{suffix}")
    bed.visual(Box((0.110, 0.240, 0.040)), origin=Origin(xyz=(0.365, 0.0, 0.275)), material=dark, name="kingpin_plate")
    bed.visual(Cylinder(radius=0.045, length=0.026), origin=Origin(xyz=(0.365, 0.0, 0.253)), material=steel, name="upper_turntable")
    bed.visual(Box((0.050, 0.060, 0.075)), origin=Origin(xyz=(0.365, 0.110, 0.253)), material=dark, name="kingpin_hanger_0")
    bed.visual(Box((0.050, 0.060, 0.075)), origin=Origin(xyz=(0.365, -0.110, 0.253)), material=dark, name="kingpin_hanger_1")

    front_bolster = model.part("front_bolster")
    # The steering frame pivots about this local origin and carries both front wheels and the drawbar bracket.
    front_bolster.visual(
        Cylinder(radius=0.014, length=0.635),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_axle",
    )
    front_bolster.visual(Box((0.420, 0.050, 0.045)), origin=Origin(xyz=(0.085, 0.0, 0.035)), material=dark, name="steering_tongue")
    front_bolster.visual(Box((0.095, 0.260, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=dark, name="axle_saddle")
    front_bolster.visual(Cylinder(radius=0.041, length=0.025), origin=Origin(xyz=(0.0, 0.0, 0.077)), material=steel, name="lower_turntable")
    front_bolster.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=steel,
        name="kingpin",
    )
    front_bolster.visual(Box((0.052, 0.016, 0.090)), origin=Origin(xyz=(0.320, 0.051, 0.065)), material=steel, name="handle_clevis_cheek_0")
    front_bolster.visual(Box((0.052, 0.016, 0.090)), origin=Origin(xyz=(0.320, -0.051, 0.065)), material=steel, name="handle_clevis_cheek_1")
    front_bolster.visual(Box((0.070, 0.112, 0.025)), origin=Origin(xyz=(0.310, 0.0, 0.018)), material=steel, name="clevis_base")
    front_bolster.visual(
        Cylinder(radius=0.012, length=0.125),
        origin=Origin(xyz=(0.320, 0.0, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_pin",
    )

    def add_wheel(name: str, side: int) -> object:
        wheel = model.part(name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=steel,
            name="spoked_wheel",
        )
        inner_y = -0.041 * side
        outer_y = 0.043 * side
        wheel.visual(
            Cylinder(radius=0.018, length=0.092),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="through_hub",
        )
        wheel.visual(
            Cylinder(radius=0.034, length=0.014),
            origin=Origin(xyz=(0.0, inner_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="inner_bearing",
        )
        wheel.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(xyz=(0.0, outer_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="outer_axle_nut",
        )
        return wheel

    rear_wheel_0 = add_wheel("rear_wheel_0", 1)
    rear_wheel_1 = add_wheel("rear_wheel_1", -1)
    front_wheel_0 = add_wheel("front_wheel_0", 1)
    front_wheel_1 = add_wheel("front_wheel_1", -1)

    handle = model.part("pull_handle")
    handle.visual(
        Cylinder(radius=0.016, length=0.760),
        origin=Origin(xyz=(0.405, 0.0, 0.048), rpy=(0.0, math.pi / 2.0 - 0.12, 0.0)),
        material=steel,
        name="handle_tube",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.300),
        origin=Origin(xyz=(0.755, 0.0, 0.092), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_grip",
    )
    handle.visual(Box((0.075, 0.042, 0.060)), origin=Origin(xyz=(0.018, 0.0, 0.0)), material=steel, name="hinge_eye")

    model.articulation(
        "bed_to_front_bolster",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=front_bolster,
        origin=Origin(xyz=(0.365, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "bed_to_rear_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=bed,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.385, 0.365, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "bed_to_rear_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=bed,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.385, -0.365, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "front_bolster_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=front_bolster,
        child=front_wheel_0,
        origin=Origin(xyz=(0.0, 0.365, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "front_bolster_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=front_bolster,
        child=front_wheel_1,
        origin=Origin(xyz=(0.0, -0.365, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "front_bolster_to_handle",
        ArticulationType.REVOLUTE,
        parent=front_bolster,
        child=handle,
        origin=Origin(xyz=(0.320, 0.0, 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.60, upper=0.95),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bed = object_model.get_part("cargo_bed")
    front_bolster = object_model.get_part("front_bolster")
    handle = object_model.get_part("pull_handle")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    steering = object_model.get_articulation("bed_to_front_bolster")
    handle_pitch = object_model.get_articulation("front_bolster_to_handle")

    ctx.allow_overlap(
        bed,
        front_bolster,
        elem_a="kingpin_plate",
        elem_b="kingpin",
        reason="The vertical steering kingpin intentionally passes through the wagon frame plate.",
    )
    ctx.allow_overlap(
        bed,
        front_bolster,
        elem_a="upper_turntable",
        elem_b="kingpin",
        reason="The same steering kingpin intentionally passes through the round turntable washer.",
    )
    ctx.allow_overlap(
        front_bolster,
        handle,
        elem_a="handle_pin",
        elem_b="hinge_eye",
        reason="The handle hinge pin is intentionally captured through the handle eye.",
    )
    ctx.expect_overlap(
        bed,
        front_bolster,
        axes="xy",
        elem_a="kingpin_plate",
        elem_b="kingpin",
        min_overlap=0.025,
        name="kingpin is captured by frame plate",
    )
    ctx.expect_overlap(
        bed,
        front_bolster,
        axes="xy",
        elem_a="upper_turntable",
        elem_b="kingpin",
        min_overlap=0.025,
        name="kingpin is captured by turntable washer",
    )
    ctx.expect_overlap(
        front_bolster,
        handle,
        axes="y",
        elem_a="handle_pin",
        elem_b="hinge_eye",
        min_overlap=0.035,
        name="handle pin spans the hinge eye",
    )
    ctx.expect_gap(
        bed,
        rear_wheel_0,
        axis="z",
        positive_elem="wood_floor",
        negative_elem="tire",
        min_gap=0.001,
        max_gap=0.020,
        name="cargo floor clears rear tire",
    )
    ctx.expect_overlap(
        front_bolster,
        front_wheel_0,
        axes="y",
        elem_a="front_axle",
        elem_b="inner_bearing",
        min_overlap=0.0003,
        name="front axle meets wheel bearing",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(handle, elem="rubber_grip")
    rest_grip_z = None if rest_grip_aabb is None else (rest_grip_aabb[0][2] + rest_grip_aabb[1][2]) / 2.0
    with ctx.pose({steering: 0.55, handle_pitch: 0.45}):
        steered_wheel = ctx.part_world_position(front_wheel_0)
        raised_grip_aabb = ctx.part_element_world_aabb(handle, elem="rubber_grip")
        raised_grip_z = None if raised_grip_aabb is None else (raised_grip_aabb[0][2] + raised_grip_aabb[1][2]) / 2.0

    ctx.check(
        "steering yaw moves front wheel",
        steered_wheel is not None and abs(steered_wheel[0] - 0.365) > 0.015,
        details=f"front wheel position after steering={steered_wheel}",
    )
    ctx.check(
        "handle pitch raises grip assembly",
        rest_grip_z is not None and raised_grip_z is not None and raised_grip_z > rest_grip_z + 0.010,
        details=f"rest_grip_z={rest_grip_z}, raised_grip_z={raised_grip_z}",
    )

    return ctx.report()


object_model = build_object_model()
