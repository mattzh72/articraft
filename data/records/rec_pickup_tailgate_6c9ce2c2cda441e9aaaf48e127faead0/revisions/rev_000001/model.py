from __future__ import annotations

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
    model = ArticulatedObject(name="pickup_tailgate")

    body_red = model.material("deep_red_paint", rgba=(0.56, 0.04, 0.035, 1.0))
    red_shadow = model.material("recessed_red_shadow", rgba=(0.36, 0.025, 0.025, 1.0))
    bedliner = model.material("black_bedliner", rgba=(0.035, 0.038, 0.040, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.020, 0.022, 0.024, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.62, 1.0))
    dull_steel = model.material("dark_zinc_hardware", rgba=(0.25, 0.26, 0.26, 1.0))
    lens_red = model.material("taillight_red", rgba=(0.95, 0.04, 0.025, 1.0))
    lens_white = model.material("reverse_lens", rgba=(0.90, 0.88, 0.78, 0.82))
    plate_white = model.material("plate_white", rgba=(0.92, 0.93, 0.88, 1.0))
    plate_blue = model.material("plate_blue", rgba=(0.08, 0.16, 0.55, 1.0))

    truck_body = model.part("truck_body")
    # Bed tub and rear opening.  These boxes deliberately overlap within the
    # same fixed body part so the assembly reads as welded truck sheet metal.
    truck_body.visual(
        Box((1.92, 1.16, 0.075)),
        origin=Origin(xyz=(0.0, 0.02, 0.605)),
        material=body_red,
        name="bed_floor_shell",
    )
    truck_body.visual(
        Box((1.82, 1.08, 0.030)),
        origin=Origin(xyz=(0.0, 0.05, 0.665)),
        material=bedliner,
        name="bedliner_floor",
    )
    for side, x in (("side_0", -0.975), ("side_1", 0.975)):
        truck_body.visual(
            Box((0.115, 1.20, 0.72)),
            origin=Origin(xyz=(x, 0.05, 0.955)),
            material=body_red,
            name=f"{side}_bed_wall",
        )
        truck_body.visual(
            Box((0.100, 1.14, 0.045)),
            origin=Origin(xyz=(x, 0.05, 1.335)),
            material=black_plastic,
            name=f"{side}_bed_rail_cap",
        )
        truck_body.visual(
            Box((0.070, 1.08, 0.600)),
            origin=Origin(xyz=(x * 0.985, 0.05, 0.975)),
            material=bedliner,
            name=f"{side}_inner_bedliner",
        )

    truck_body.visual(
        Box((1.88, 0.120, 0.120)),
        origin=Origin(xyz=(0.0, -0.580, 0.470)),
        material=body_red,
        name="rear_lower_sill",
    )
    truck_body.visual(
        Box((1.88, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.550, 0.545)),
        material=bedliner,
        name="sill_black_scuff",
    )
    truck_body.visual(
        Box((2.08, 0.170, 0.160)),
        origin=Origin(xyz=(0.0, -0.760, 0.335)),
        material=steel,
        name="rear_bumper",
    )
    truck_body.visual(
        Box((0.520, 0.135, 0.038)),
        origin=Origin(xyz=(0.0, -0.860, 0.425)),
        material=black_plastic,
        name="bumper_step_pad",
    )
    for x in (-0.52, 0.52):
        truck_body.visual(
            Box((0.080, 0.250, 0.160)),
            origin=Origin(xyz=(x, -0.670, 0.430)),
            material=dull_steel,
            name=f"bumper_support_{0 if x < 0 else 1}",
        )

    for side_index, x in enumerate((-0.955, 0.955)):
        truck_body.visual(
            Box((0.130, 0.165, 0.780)),
            origin=Origin(xyz=(x, -0.555, 0.955)),
            material=body_red,
            name=f"rear_pillar_{side_index}",
        )
        truck_body.visual(
            Box((0.060, 0.018, 0.285)),
            origin=Origin(xyz=(x, -0.646, 1.030)),
            material=lens_red,
            name=f"taillight_lens_{side_index}",
        )
        truck_body.visual(
            Box((0.052, 0.020, 0.070)),
            origin=Origin(xyz=(x, -0.650, 1.010)),
            material=lens_white,
            name=f"reverse_lens_{side_index}",
        )
        truck_body.visual(
            Box((0.055, 0.030, 0.115)),
            origin=Origin(xyz=(x * 0.915, -0.630, 1.150)),
            material=dull_steel,
            name=f"latch_striker_{side_index}",
        )
        truck_body.visual(
            Box((0.080, 0.070, 0.085)),
            origin=Origin(xyz=((1.0 if x > 0.0 else -1.0) * 0.890, -0.610, 0.585)),
            material=dull_steel,
            name=f"hinge_mount_block_{side_index}",
        )
        truck_body.visual(
            Cylinder(radius=0.020, length=0.095),
            origin=Origin(
                xyz=((1.0 if x > 0.0 else -1.0) * 0.875, -0.610, 0.585),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_pin_stub_{side_index}",
        )

    truck_body.inertial = Inertial.from_geometry(
        Box((2.10, 1.25, 1.05)),
        mass=180.0,
        origin=Origin(xyz=(0.0, -0.05, 0.85)),
    )

    tailgate = model.part("tailgate")
    # The tailgate part frame is the hinge pin line.  Closed geometry extends
    # upward in +Z and outward/rearward along -Y, so +q about +X drops it down.
    tailgate.visual(
        Box((1.68, 0.050, 0.725)),
        origin=Origin(xyz=(0.0, -0.026, 0.435)),
        material=body_red,
        name="outer_panel_shell",
    )
    tailgate.visual(
        Box((1.54, 0.012, 0.455)),
        origin=Origin(xyz=(0.0, -0.055, 0.475)),
        material=red_shadow,
        name="stamped_recess_floor",
    )
    # Raised perimeter and inner stamp beads.
    tailgate.visual(
        Box((1.70, 0.020, 0.052)),
        origin=Origin(xyz=(0.0, -0.058, 0.795)),
        material=body_red,
        name="upper_stamp_rail",
    )
    tailgate.visual(
        Box((1.68, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, -0.058, 0.090)),
        material=body_red,
        name="lower_stamp_rail",
    )
    for side_index, x in enumerate((-0.835, 0.835)):
        tailgate.visual(
            Box((0.060, 0.020, 0.720)),
            origin=Origin(xyz=(x, -0.058, 0.435)),
            material=body_red,
            name=f"side_stamp_rail_{side_index}",
        )
        tailgate.visual(
            Box((0.028, 0.028, 0.540)),
            origin=Origin(xyz=(x * 0.965, -0.030, 0.440)),
            material=body_red,
            name=f"folded_side_flange_{side_index}",
        )

    tailgate.visual(
        Box((1.28, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, -0.058, 0.670)),
        material=body_red,
        name="inner_upper_bead",
    )
    tailgate.visual(
        Box((1.28, 0.022, 0.043)),
        origin=Origin(xyz=(0.0, -0.058, 0.292)),
        material=body_red,
        name="inner_lower_bead",
    )
    for side_index, x in enumerate((-0.660, 0.660)):
        tailgate.visual(
            Box((0.042, 0.022, 0.405)),
            origin=Origin(xyz=(x, -0.058, 0.480)),
            material=body_red,
            name=f"inner_vertical_bead_{side_index}",
        )

    # Top cap and rolled bottom edge give the panel thickness and pickup-body
    # character.
    tailgate.visual(
        Box((1.74, 0.110, 0.055)),
        origin=Origin(xyz=(0.0, -0.026, 0.818)),
        material=black_plastic,
        name="top_cap",
    )
    tailgate.visual(
        Cylinder(radius=0.030, length=1.72),
        origin=Origin(
            xyz=(0.0, -0.079, 0.828),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_plastic,
        name="rounded_top_lip",
    )
    tailgate.visual(
        Cylinder(radius=0.034, length=1.54),
        origin=Origin(
            xyz=(0.0, -0.020, 0.036),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_red,
        name="bottom_rolled_tube",
    )

    # Center handle pocket, license plate, latch bosses, and hinge knuckles.
    tailgate.visual(
        Box((0.395, 0.018, 0.105)),
        origin=Origin(xyz=(0.0, -0.073, 0.705)),
        material=black_plastic,
        name="handle_pocket",
    )
    tailgate.visual(
        Box((0.440, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.075, 0.642)),
        material=body_red,
        name="handle_lower_brow",
    )
    tailgate.visual(
        Box((0.410, 0.013, 0.190)),
        origin=Origin(xyz=(0.0, -0.073, 0.220)),
        material=black_plastic,
        name="license_recess",
    )
    tailgate.visual(
        Box((0.330, 0.010, 0.145)),
        origin=Origin(xyz=(0.0, -0.081, 0.220)),
        material=plate_white,
        name="license_plate",
    )
    tailgate.visual(
        Box((0.330, 0.011, 0.028)),
        origin=Origin(xyz=(0.0, -0.088, 0.282)),
        material=plate_blue,
        name="plate_top_band",
    )
    for side_index, x in enumerate((-0.765, 0.765)):
        tailgate.visual(
            Box((0.080, 0.050, 0.105)),
            origin=Origin(xyz=(x, -0.030, 0.575)),
            material=dull_steel,
            name=f"side_latch_box_{side_index}",
        )
        tailgate.visual(
            Cylinder(radius=0.020, length=0.125),
            origin=Origin(
                xyz=(x, -0.020, 0.036),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_knuckle_{side_index}",
        )
        tailgate.visual(
            Box((0.072, 0.030, 0.070)),
            origin=Origin(xyz=(x, -0.030, 0.090)),
            material=dull_steel,
            name=f"hinge_leaf_{side_index}",
        )

    tailgate.inertial = Inertial.from_geometry(
        Box((1.74, 0.12, 0.86)),
        mass=34.0,
        origin=Origin(xyz=(0.0, -0.030, 0.430)),
    )

    hinge = model.articulation(
        "body_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=truck_body,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.610, 0.550)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(94.0),
        ),
    )
    hinge.meta["qc_samples"] = [0.0, math.radians(50.0), math.radians(94.0)]

    handle = model.part("handle")
    handle.visual(
        Box((0.285, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.010, -0.031)),
        material=dark_rubber,
        name="pull_handle",
    )
    handle.visual(
        Box((0.255, 0.011, 0.010)),
        origin=Origin(xyz=(0.0, -0.022, -0.057)),
        material=black_plastic,
        name="handle_finger_lip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.29, 0.025, 0.060)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.010, -0.030)),
    )
    model.articulation(
        "tailgate_to_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=handle,
        origin=Origin(xyz=(0.0, -0.082, 0.735)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.5,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(26.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("truck_body")
    tailgate = object_model.get_part("tailgate")
    handle = object_model.get_part("handle")
    hinge = object_model.get_articulation("body_to_tailgate")
    handle_joint = object_model.get_articulation("tailgate_to_handle")

    ctx.check("truck body present", body is not None, "Expected fixed truck-body rear structure.")
    ctx.check("tailgate present", tailgate is not None, "Expected articulated tailgate panel.")
    ctx.check("handle present", handle is not None, "Expected separate movable tailgate handle.")
    if body is None or tailgate is None or handle is None or hinge is None or handle_joint is None:
        return ctx.report()

    tailgate_aabb = ctx.part_world_aabb(tailgate)
    ctx.check("tailgate aabb", tailgate_aabb is not None, "Expected tailgate world bounds.")
    if tailgate_aabb is not None:
        mins, maxs = tailgate_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("tailgate pickup scale", 1.65 <= size[0] <= 1.85 and 0.78 <= size[2] <= 0.92, f"size={size!r}")

    # Closed tailgate sits between the rear pillars, overlaps the truck opening
    # laterally, and remains above the rear sill rather than intersecting it.
    ctx.expect_overlap(
        tailgate,
        body,
        axes="x",
        elem_a="outer_panel_shell",
        elem_b="rear_lower_sill",
        min_overlap=1.50,
        name="closed tailgate spans rear opening",
    )
    ctx.expect_gap(
        tailgate,
        body,
        axis="z",
        positive_elem="bottom_rolled_tube",
        negative_elem="rear_lower_sill",
        min_gap=0.010,
        max_gap=0.045,
        name="bottom roll clears rear sill",
    )

    closed_aabb = ctx.part_world_aabb(tailgate)
    with ctx.pose({hinge: math.radians(94.0)}):
        dropped_aabb = ctx.part_world_aabb(tailgate)
    ctx.check(
        "tailgate drops rearward and down",
        closed_aabb is not None
        and dropped_aabb is not None
        and dropped_aabb[0][1] < closed_aabb[0][1] - 0.40
        and dropped_aabb[1][2] < closed_aabb[1][2] - 0.45,
        details=f"closed={closed_aabb!r}, dropped={dropped_aabb!r}",
    )

    handle_rest = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: math.radians(26.0)}):
        handle_pulled = ctx.part_world_aabb(handle)
    ctx.check(
        "handle pulls outward",
        handle_rest is not None
        and handle_pulled is not None
        and handle_pulled[0][1] < handle_rest[0][1] - 0.002,
        details=f"rest={handle_rest!r}, pulled={handle_pulled!r}",
    )

    return ctx.report()


object_model = build_object_model()
