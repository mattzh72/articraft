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
    TireGeometry,
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
    model = ArticulatedObject(name="stackable_rolling_toolbox_base")

    body_mat = model.material("charcoal_rugged_polymer", rgba=(0.10, 0.11, 0.11, 1.0))
    lid_mat = model.material("black_lid_polymer", rgba=(0.035, 0.038, 0.04, 1.0))
    rib_mat = model.material("molded_edge_highlights", rgba=(0.16, 0.17, 0.16, 1.0))
    latch_mat = model.material("yellow_composite_latches", rgba=(0.95, 0.66, 0.08, 1.0))
    button_mat = model.material("red_release_button", rgba=(0.78, 0.05, 0.035, 1.0))
    metal_mat = model.material("brushed_steel_hardware", rgba=(0.58, 0.60, 0.60, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    rim_mat = model.material("dark_gray_wheel_rim", rgba=(0.20, 0.21, 0.21, 1.0))

    body = model.part("body")

    # Large open lower bin: bottom pan and four thick walls leave a visible open tub
    # when the rear-hinged lid is lifted.
    body.visual(Box((0.70, 0.40, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.110)), material=body_mat, name="bottom_pan")
    body.visual(Box((0.035, 0.42, 0.340)), origin=Origin(xyz=(-0.3525, 0.0, 0.300)), material=body_mat, name="front_wall")
    body.visual(Box((0.035, 0.42, 0.340)), origin=Origin(xyz=(0.3525, 0.0, 0.300)), material=body_mat, name="rear_wall")
    body.visual(Box((0.72, 0.035, 0.340)), origin=Origin(xyz=(0.0, 0.2025, 0.300)), material=body_mat, name="side_wall_0")
    body.visual(Box((0.72, 0.035, 0.340)), origin=Origin(xyz=(0.0, -0.2025, 0.300)), material=body_mat, name="side_wall_1")

    # Rugged molded break lines, stacking ledges, and side catch pads.
    body.visual(Box((0.74, 0.035, 0.030)), origin=Origin(xyz=(0.0, 0.225, 0.465)), material=rib_mat, name="upper_ledge_0")
    body.visual(Box((0.74, 0.035, 0.030)), origin=Origin(xyz=(0.0, -0.225, 0.465)), material=rib_mat, name="upper_ledge_1")
    body.visual(Box((0.035, 0.44, 0.030)), origin=Origin(xyz=(-0.372, 0.0, 0.465)), material=rib_mat, name="front_ledge")
    body.visual(Box((0.025, 0.44, 0.030)), origin=Origin(xyz=(0.345, 0.0, 0.465)), material=rib_mat, name="rear_ledge")
    body.visual(Box((0.020, 0.34, 0.028)), origin=Origin(xyz=(-0.379, 0.0, 0.230)), material=rib_mat, name="front_lower_rib")
    body.visual(Box((0.020, 0.34, 0.028)), origin=Origin(xyz=(-0.379, 0.0, 0.390)), material=rib_mat, name="front_upper_rib")
    body.visual(Box((0.025, 0.080, 0.045)), origin=Origin(xyz=(-0.110, 0.242, 0.302)), material=rib_mat, name="side_catch_0")
    body.visual(Box((0.025, 0.080, 0.045)), origin=Origin(xyz=(-0.110, -0.242, 0.302)), material=rib_mat, name="side_catch_1")
    body.visual(Box((0.13, 0.08, 0.045)), origin=Origin(xyz=(-0.255, 0.145, 0.058)), material=rib_mat, name="front_foot_0")
    body.visual(Box((0.13, 0.08, 0.045)), origin=Origin(xyz=(-0.255, -0.145, 0.058)), material=rib_mat, name="front_foot_1")

    # Rear axle and stout molded wheel pockets.
    body.visual(
        Cylinder(radius=0.012, length=0.58),
        origin=Origin(xyz=(0.255, 0.0, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="rear_axle",
    )
    body.visual(Box((0.10, 0.035, 0.17)), origin=Origin(xyz=(0.255, 0.221, 0.132)), material=body_mat, name="wheel_pocket_0")
    body.visual(Box((0.10, 0.035, 0.17)), origin=Origin(xyz=(0.255, -0.221, 0.132)), material=body_mat, name="wheel_pocket_1")

    # Twin fixed rear guide rails for the telescoping pull handle.
    for idx, y in enumerate((0.145, -0.145)):
        body.visual(Box((0.035, 0.060, 0.500)), origin=Origin(xyz=(0.412, y, 0.325)), material=body_mat, name=f"rear_rail_{idx}")
        body.visual(Box((0.070, 0.080, 0.045)), origin=Origin(xyz=(0.382, y, 0.145)), material=body_mat, name=f"rail_lower_mount_{idx}")
        body.visual(Box((0.060, 0.080, 0.045)), origin=Origin(xyz=(0.450, y, 0.500)), material=body_mat, name=f"rail_upper_mount_{idx}")
        body.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.432, y, 0.548), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"rail_pin_{idx}",
        )

    # Hinge knuckles mounted to the body, leaving a clear center gap for the lid knuckle.
    for idx, y in enumerate((0.155, -0.155)):
        body.visual(
            Cylinder(radius=0.023, length=0.090),
            origin=Origin(xyz=(0.392, y, 0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"body_hinge_barrel_{idx}",
        )
        body.visual(Box((0.055, 0.105, 0.026)), origin=Origin(xyz=(0.372, y, 0.468)), material=metal_mat, name=f"body_hinge_leaf_{idx}")

    # Rear-hinged lid, authored in a hinge-line child frame.  At q=0 it extends
    # forward along local -X with a deliberate visible seam above the bin rim.
    lid = model.part("lid")
    lid.visual(Box((0.720, 0.460, 0.065)), origin=Origin(xyz=(-0.390, 0.0, 0.034)), material=lid_mat, name="lid_shell")
    lid.visual(Box((0.540, 0.300, 0.026)), origin=Origin(xyz=(-0.390, 0.0, 0.0785)), material=rib_mat, name="stacking_recess")
    lid.visual(Box((0.610, 0.034, 0.028)), origin=Origin(xyz=(-0.385, 0.183, 0.0805)), material=rib_mat, name="top_rib_0")
    lid.visual(Box((0.610, 0.034, 0.028)), origin=Origin(xyz=(-0.385, -0.183, 0.0805)), material=rib_mat, name="top_rib_1")
    lid.visual(Box((0.055, 0.365, 0.078)), origin=Origin(xyz=(-0.756, 0.0, 0.034)), material=lid_mat, name="front_nose")
    lid.visual(Box((0.014, 0.220, 0.052)), origin=Origin(xyz=(-0.787, 0.0, 0.033)), material=rib_mat, name="button_pocket")
    lid.visual(
        Cylinder(radius=0.022, length=0.150),
        origin=Origin(xyz=(0.005, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="lid_hinge_barrel",
    )
    lid.visual(Box((0.070, 0.170, 0.023)), origin=Origin(xyz=(-0.035, 0.0, 0.012)), material=metal_mat, name="lid_hinge_leaf")

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.392, 0.0, 0.485)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.82),
    )

    # Push-button latch in the front nose; positive travel pushes it inward.
    button = model.part("front_button")
    button.visual(Box((0.038, 0.170, 0.044)), origin=Origin(xyz=(-0.016, 0.0, 0.0)), material=button_mat, name="button_cap")
    button.visual(Box((0.030, 0.110, 0.028)), origin=Origin(xyz=(0.015, 0.0, 0.0)), material=button_mat, name="button_stem")
    button_slide = model.articulation(
        "lid_to_front_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=button,
        origin=Origin(xyz=(-0.787, 0.0, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.30, lower=0.0, upper=0.026),
    )

    # Two side latch flaps are mounted on their own pivots on the lid sides.
    for idx, (y, outward, axis_x) in enumerate(((0.248, 1.0, 1.0), (-0.248, -1.0, -1.0))):
        latch = model.part(f"side_latch_{idx}")
        latch.visual(
            Cylinder(radius=0.018, length=0.130),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=latch_mat,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.115, 0.023, 0.170)),
            origin=Origin(xyz=(0.0, outward * 0.017, -0.073)),
            material=latch_mat,
            name="latch_flap",
        )
        latch.visual(
            Box((0.080, 0.032, 0.032)),
            origin=Origin(xyz=(0.0, outward * 0.020, -0.170)),
            material=latch_mat,
            name="catch_hook",
        )
        model.articulation(
            f"lid_to_side_latch_{idx}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(-0.455, y, 0.030)),
            axis=(axis_x, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=0.0, upper=1.20),
        )

    # Telescoping U-handle sliding on the twin rear guide rails.
    handle = model.part("handle")
    for idx, y in enumerate((0.145, -0.145)):
        handle.visual(Box((0.026, 0.026, 0.720)), origin=Origin(xyz=(0.0, y, -0.020)), material=metal_mat, name=f"handle_tube_{idx}")
    handle.visual(Box((0.040, 0.360, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.360)), material=metal_mat, name="handle_crossbar")
    handle.visual(Box((0.052, 0.245, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.380)), material=rubber_mat, name="grip_sleeve")
    handle_slide = model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.455, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=0.450),
    )

    # Two large continuously spinning rear wheels with separate tire and rim visuals.
    tire_meshes = []
    rim_meshes = []
    for idx in range(2):
        tire_meshes.append(
            mesh_from_geometry(
                TireGeometry(
                    0.095,
                    0.055,
                    inner_radius=0.064,
                    tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.55),
                    sidewall=TireSidewall(style="square", bulge=0.02),
                    shoulder=TireShoulder(width=0.006, radius=0.003),
                ),
                f"wheel_tire_{idx}",
            )
        )
        rim_meshes.append(
            mesh_from_geometry(
                WheelGeometry(
                    0.063,
                    0.050,
                    rim=WheelRim(inner_radius=0.040, flange_height=0.006, flange_thickness=0.003),
                    hub=WheelHub(
                        radius=0.021,
                        width=0.040,
                        cap_style="domed",
                        bolt_pattern=BoltPattern(count=5, circle_diameter=0.028, hole_diameter=0.0035),
                    ),
                    face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
                    bore=WheelBore(style="round", diameter=0.018),
                ),
                f"wheel_rim_{idx}",
            )
        )

    for idx, y in enumerate((0.272, -0.272)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_meshes[idx], origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber_mat, name="tire")
        wheel.visual(rim_meshes[idx], origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rim_mat, name="rim")
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(0.255, y, 0.095)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    button = object_model.get_part("front_button")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("body_to_handle")
    button_slide = object_model.get_articulation("lid_to_front_button")

    ctx.allow_overlap(
        body,
        wheel_0,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The steel axle intentionally passes through the wheel hub/bore so the wheel reads as captured on the axle.",
    )
    ctx.allow_overlap(
        body,
        wheel_1,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The steel axle intentionally passes through the wheel hub/bore so the wheel reads as captured on the axle.",
    )
    ctx.allow_overlap(
        lid,
        button,
        elem_a="button_pocket",
        elem_b="button_cap",
        reason="The spring-loaded push button is intentionally seated slightly into the molded pocket in the lid nose.",
    )
    ctx.allow_overlap(
        lid,
        button,
        elem_a="front_nose",
        elem_b="button_stem",
        reason="The push-button stem intentionally slides inside the lid nose shell as the latch release is pressed.",
    )
    ctx.allow_overlap(
        lid,
        button,
        elem_a="button_pocket",
        elem_b="button_stem",
        reason="The button stem intentionally passes through the molded pocket throat on its way into the lid nose.",
    )
    for idx in range(2):
        ctx.allow_overlap(
            body,
            handle,
            elem_a=f"rail_upper_mount_{idx}",
            elem_b=f"handle_tube_{idx}",
            reason="The telescoping handle tube is captured by the molded upper guide collar, represented as a solid collar proxy.",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="front_ledge",
        min_gap=0.002,
        max_gap=0.040,
        name="closed lid has a visible body seam",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.32, name="closed lid covers the lower bin footprint")
    ctx.expect_overlap(body, wheel_0, axes="y", elem_a="rear_axle", elem_b="rim", min_overlap=0.020, name="wheel 0 retained on axle")
    ctx.expect_overlap(body, wheel_1, axes="y", elem_a="rear_axle", elem_b="rim", min_overlap=0.020, name="wheel 1 retained on axle")
    ctx.expect_gap(lid, button, axis="x", elem_a="button_pocket", elem_b="button_cap", max_penetration=0.024, name="button is shallowly seated in the nose pocket")
    ctx.expect_within(button, lid, axes="yz", inner_elem="button_cap", outer_elem="button_pocket", margin=0.020, name="button stays centered in the pocket opening")
    ctx.expect_within(button, lid, axes="yz", inner_elem="button_stem", outer_elem="button_pocket", margin=0.002, name="button stem passes through the pocket throat")
    ctx.expect_within(button, lid, axes="yz", inner_elem="button_stem", outer_elem="front_nose", margin=0.002, name="button stem rides inside the lid nose")
    for idx in range(2):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem=f"handle_tube_{idx}",
            outer_elem=f"rail_upper_mount_{idx}",
            margin=0.002,
            name=f"handle tube {idx} is centered in upper guide collar",
        )

    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.450}):
        extended_handle = ctx.part_world_position(handle)
        ctx.expect_overlap(handle, body, axes="z", elem_a="handle_tube_0", elem_b="rear_rail_0", min_overlap=0.045, name="extended handle tube 0 remains in rear rail")
        ctx.expect_overlap(handle, body, axes="z", elem_a="handle_tube_1", elem_b="rear_rail_1", min_overlap=0.045, name="extended handle tube 1 remains in rear rail")
    ctx.check(
        "telescoping handle slides upward",
        rest_handle is not None and extended_handle is not None and extended_handle[2] > rest_handle[2] + 0.40,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear lid hinge opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.026}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "front push button translates inward",
        rest_button is not None and pressed_button is not None and pressed_button[0] > rest_button[0] + 0.020,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    required_joint_types = {
        "body_to_lid": ArticulationType.REVOLUTE,
        "body_to_handle": ArticulationType.PRISMATIC,
        "lid_to_front_button": ArticulationType.PRISMATIC,
        "lid_to_side_latch_0": ArticulationType.REVOLUTE,
        "lid_to_side_latch_1": ArticulationType.REVOLUTE,
        "body_to_wheel_0": ArticulationType.CONTINUOUS,
        "body_to_wheel_1": ArticulationType.CONTINUOUS,
    }
    for name, expected_type in required_joint_types.items():
        joint = object_model.get_articulation(name)
        ctx.check(f"{name} has requested articulation", joint.articulation_type == expected_type, details=str(joint.articulation_type))

    return ctx.report()


object_model = build_object_model()
