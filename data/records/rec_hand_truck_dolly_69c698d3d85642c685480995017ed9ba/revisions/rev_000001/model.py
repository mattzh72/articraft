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
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="appliance_hand_truck")

    frame_red = model.material("powder_coated_red", color=(0.75, 0.06, 0.035, 1.0))
    dark_rubber = model.material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    zinc = model.material("zinc_plated_steel", color=(0.70, 0.72, 0.70, 1.0))
    dull_steel = model.material("dull_steel", color=(0.48, 0.50, 0.50, 1.0))
    grip_black = model.material("black_grips", color=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")

    def cyl(
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        material: Material | str = frame_red,
    ) -> None:
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    # Two tall tubular uprights, with welded cross members, form the main back frame.
    for y in (-0.24, 0.24):
        cyl("upright_0" if y < 0 else "upright_1", 0.022, 1.34, (0.0, y, 0.78))
        cyl(
            "lower_side_stay_0" if y < 0 else "lower_side_stay_1",
            0.016,
            0.19,
            (-0.06, y, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        )

    for z, radius, name in (
        (0.20, 0.020, "base_crossbar"),
        (0.60, 0.018, "middle_crossbar"),
        (1.04, 0.018, "strap_crossbar"),
        (1.34, 0.020, "top_crossbar"),
    ):
        cyl(name, radius, 0.54, (0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0))

    # A single bent top handle bridges the uprights and reads as a continuous tube.
    handle_geom = tube_from_spline_points(
        [
            (0.0, -0.24, 1.38),
            (-0.02, -0.24, 1.52),
            (-0.03, -0.13, 1.60),
            (-0.03, 0.13, 1.60),
            (-0.02, 0.24, 1.52),
            (0.0, 0.24, 1.38),
        ],
        radius=0.022,
        samples_per_segment=14,
        radial_segments=20,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(handle_geom, "bent_top_handle"),
        material=frame_red,
        name="bent_top_handle",
    )

    # Black hand grips on the top handle are separate sleeves, but welded/pressed onto the handle tube.
    cyl(
        "grip_0",
        0.026,
        0.16,
        (-0.03, -0.16, 1.60),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=grip_black,
    )
    cyl(
        "grip_1",
        0.026,
        0.16,
        (-0.03, 0.16, 1.60),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=grip_black,
    )

    # Wheel axle and the hinge pin for the folding nose plate are exposed steel.
    cyl(
        "wheel_axle",
        0.019,
        0.82,
        (-0.13, 0.0, 0.18),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=dull_steel,
    )
    cyl(
        "nose_hinge_pin",
        0.012,
        0.68,
        (0.055, 0.0, 0.09),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=dull_steel,
    )

    # Welded side plates and gussets carry the hinge pin forward from the uprights.
    for y in (-0.305, 0.305):
        side = "0" if y < 0 else "1"
        frame.visual(
            Box((0.10, 0.018, 0.08)),
            origin=Origin(xyz=(0.025, y, 0.105)),
            material=zinc,
            name=f"hinge_cheek_{side}",
        )
        frame.visual(
            Box((0.13, 0.014, 0.050)),
            origin=Origin(xyz=(-0.065, y * 0.79, 0.205), rpy=(0.0, 0.42, 0.0)),
            material=frame_red,
            name=f"axle_gusset_{side}",
        )
        hinge_strut = wire_from_points(
            [
                (0.012, y * 0.79, 0.175),
                (0.038, y * 0.92, 0.158),
                (0.050, y, 0.135),
            ],
            radius=0.010,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.012,
            corner_segments=6,
        )
        frame.visual(
            mesh_from_geometry(hinge_strut, f"hinge_strut_{side}"),
            material=frame_red,
            name=f"hinge_strut_{side}",
        )

    # Pair of strap hooks on the front of the frame for appliance-retaining straps.
    for i, y in enumerate((-0.15, 0.15)):
        hook = wire_from_points(
            [
                (0.012, y, 1.04),
                (0.070, y, 1.04),
                (0.078, y, 0.985),
                (0.050, y, 0.970),
            ],
            radius=0.009,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.018,
            corner_segments=8,
        )
        frame.visual(
            mesh_from_geometry(hook, f"strap_hook_{i}"),
            material=zinc,
            name=f"strap_hook_{i}",
        )

    # Folding toe/nose plate: child frame is the transverse hinge line.
    nose = model.part("nose_plate")
    nose.visual(
        Box((0.44, 0.64, 0.022)),
        origin=Origin(xyz=(0.24, 0.0, -0.025)),
        material=zinc,
        name="plate_deck",
    )
    nose.visual(
        Box((0.035, 0.62, 0.050)),
        origin=Origin(xyz=(0.455, 0.0, 0.000), rpy=(0.0, -0.18, 0.0)),
        material=zinc,
        name="front_lip",
    )
    for i, x in enumerate((0.11, 0.18, 0.25, 0.32, 0.39)):
        nose.visual(
            Box((0.012, 0.56, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.011)),
            material=dull_steel,
            name=f"traction_rib_{i}",
        )
    for i, y in enumerate((-0.20, 0.20)):
        nose.visual(
            Cylinder(radius=0.022, length=0.155),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"hinge_sleeve_{i}",
        )
        nose.visual(
            Box((0.105, 0.135, 0.012)),
            origin=Origin(xyz=(0.052, y, -0.014)),
            material=zinc,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "frame_to_nose_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=nose,
        origin=Origin(xyz=(0.055, 0.0, 0.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=1.6, lower=0.0, upper=math.pi / 2.0),
    )

    tire_geom = TireGeometry(
        0.180,
        0.090,
        inner_radius=0.128,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    rim_geom = WheelGeometry(
        0.132,
        0.074,
        rim=WheelRim(inner_radius=0.088, flange_height=0.010, flange_thickness=0.006, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.043,
            width=0.064,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.048, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.008, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.044),
    )
    tire_mesh = mesh_from_geometry(tire_geom, "utility_tire")
    rim_mesh = mesh_from_geometry(rim_geom, "steel_wheel_rim")

    for i, y in enumerate((-0.36, 0.36)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.0235, length=0.11),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dull_steel,
            name="bearing_sleeve",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dull_steel,
            name="rim",
        )
        model.articulation(
            f"frame_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.13, y, 0.18)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    nose = object_model.get_part("nose_plate")
    nose_joint = object_model.get_articulation("frame_to_nose_plate")

    # The folding plate knuckles intentionally wrap the fixed hinge pin.
    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1"):
        ctx.allow_overlap(
            frame,
            nose,
            elem_a="nose_hinge_pin",
            elem_b=sleeve,
            reason="The hinge sleeve is intentionally captured around the transverse nose-plate pin.",
        )
        ctx.expect_overlap(
            frame,
            nose,
            axes="y",
            elem_a="nose_hinge_pin",
            elem_b=sleeve,
            min_overlap=0.08,
            name=f"{sleeve} wraps the hinge pin along the transverse axis",
        )

    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="bearing_sleeve",
            reason="The wheel bearing sleeve is intentionally captured on the fixed axle so the wheel can spin.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="bearing_sleeve",
            min_overlap=0.09,
            name=f"{wheel_name} bearing remains on the axle",
        )

    with ctx.pose({nose_joint: 0.0}):
        ctx.expect_gap(
            nose,
            frame,
            axis="x",
            positive_elem="plate_deck",
            negative_elem="wheel_axle",
            min_gap=0.12,
            name="deployed nose plate sits forward of the wheel axle",
        )
        ctx.expect_overlap(
            nose,
            frame,
            axes="y",
            elem_a="plate_deck",
            elem_b="base_crossbar",
            min_overlap=0.45,
            name="nose plate spans most of the truck width",
        )

    folded_pos = None
    rest_deck_aabb = ctx.part_element_world_aabb(nose, elem="plate_deck")
    with ctx.pose({nose_joint: math.pi / 2.0}):
        folded_deck_aabb = ctx.part_element_world_aabb(nose, elem="plate_deck")
        if rest_deck_aabb is not None and folded_deck_aabb is not None:
            folded_pos = (
                (rest_deck_aabb[0][2] + rest_deck_aabb[1][2]) / 2.0,
                (folded_deck_aabb[0][2] + folded_deck_aabb[1][2]) / 2.0,
            )
        ctx.expect_overlap(
            nose,
            frame,
            axes="z",
            elem_a="plate_deck",
            elem_b="upright_0",
            min_overlap=0.18,
            name="folded plate lies against an upright",
        )
        ctx.expect_gap(
            nose,
            frame,
            axis="x",
            positive_elem="plate_deck",
            negative_elem="upright_0",
            min_gap=0.005,
            max_gap=0.08,
            name="folded plate stores just in front of the frame",
        )

    ctx.check(
        "nose plate hinge raises the plate for storage",
        folded_pos is not None and folded_pos[1] > folded_pos[0] + 0.10,
        details=f"deck_center_z(rest, folded)={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
