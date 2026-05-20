from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_luggage_hand_truck")

    metal = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("black_powdercoat", rgba=(0.02, 0.025, 0.025, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    plate_mat = model.material("dark_toe_plate", rgba=(0.08, 0.085, 0.08, 1.0))
    grip_mat = model.material("matte_handle_grip", rgba=(0.015, 0.015, 0.015, 1.0))

    cyl_x = (0.0, math.pi / 2.0, 0.0)

    # Root compact frame: two hollow side sleeves with rear/front cross members,
    # bottom hinge brackets, and a live axle tucked between the inset wheels.
    frame = model.part("frame")
    outer = rounded_rect_profile(0.050, 0.050, 0.006, corner_segments=6)
    inner = rounded_rect_profile(0.026, 0.026, 0.004, corner_segments=6)
    sleeve_mesh = ExtrudeWithHolesGeometry(outer, [inner], 0.580, center=True, closed=True)
    for idx, x in enumerate((-0.155, 0.155)):
        frame.visual(
            mesh_from_geometry(sleeve_mesh, f"side_sleeve_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.420)),
            material=metal,
            name=f"sleeve_{idx}",
        )

    for name, z, sx, sz in (
        ("lower_crossbar", 0.215, 0.350, 0.026),
        ("middle_crossbar", 0.455, 0.315, 0.022),
        ("top_crossbar", 0.680, 0.340, 0.026),
    ):
        frame.visual(
            Box((sx, 0.030, sz)),
            origin=Origin(xyz=(0.0, 0.035, z)),
            material=dark,
            name=name,
        )

    frame.visual(
        Box((0.210, 0.012, 0.300)),
        origin=Origin(xyz=(0.0, 0.051, 0.405)),
        material=dark,
        name="back_panel",
    )
    frame.visual(
        Box((0.360, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.075, 0.170)),
        material=dark,
        name="front_hinge_beam",
    )
    for idx, x in enumerate((-0.155, 0.155)):
        frame.visual(
            Box((0.026, 0.060, 0.030)),
            origin=Origin(xyz=(x, -0.050, 0.170)),
            material=dark,
            name=f"hinge_strut_{idx}",
        )
    for idx, x in enumerate((-0.185, 0.185)):
        frame.visual(
            Box((0.022, 0.040, 0.060)),
            origin=Origin(xyz=(x, -0.075, 0.125)),
            material=dark,
            name=f"hinge_cheek_{idx}",
        )

    frame.visual(
        Cylinder(radius=0.009, length=0.380),
        origin=Origin(xyz=(0.0, 0.043, 0.095), rpy=cyl_x),
        material=metal,
        name="axle_tube",
    )
    for idx, x in enumerate((-0.155, 0.155)):
        frame.visual(
            Box((0.026, 0.032, 0.108)),
            origin=Origin(xyz=(x, 0.039, 0.145)),
            material=dark,
            name=f"axle_drop_{idx}",
        )

    # Prismatic handle assembly: two smaller tubes live inside the guide sleeves
    # and remain engaged even at full extension.
    handle = model.part("handle")
    for idx, x in enumerate((-0.155, 0.155)):
        handle.visual(
            Cylinder(radius=0.009, length=0.680),
            origin=Origin(xyz=(x, 0.0, -0.160)),
            material=metal,
            name=f"rail_{idx}",
        )
    handle.visual(
        Cylinder(radius=0.013, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.185), rpy=cyl_x),
        material=metal,
        name="handle_bar",
    )
    handle.visual(
        Cylinder(radius=0.019, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.185), rpy=cyl_x),
        material=grip_mat,
        name="grip_sleeve",
    )

    # Hinged folding toe plate.  The rest pose is the deployed carrying shelf;
    # positive rotation folds it upward against the frame.
    toe = model.part("toe_plate")
    toe.visual(
        Cylinder(radius=0.011, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_x),
        material=dark,
        name="hinge_barrel",
    )
    toe.visual(
        Box((0.320, 0.245, 0.012)),
        origin=Origin(xyz=(0.0, -0.125, -0.006)),
        material=plate_mat,
        name="deck",
    )
    for idx, x in enumerate((-0.105, 0.0, 0.105)):
        toe.visual(
            Box((0.026, 0.205, 0.007)),
            origin=Origin(xyz=(x, -0.145, 0.0035)),
            material=dark,
            name=f"raised_rib_{idx}",
        )
    for idx, x in enumerate((-0.165, 0.165)):
        toe.visual(
            Box((0.012, 0.230, 0.034)),
            origin=Origin(xyz=(x, -0.125, 0.011)),
            material=dark,
            name=f"side_lip_{idx}",
        )
    toe.visual(
        Box((0.315, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, -0.254, 0.011)),
        material=dark,
        name="front_lip",
    )

    wheel_mesh = WheelGeometry(
        0.056,
        0.040,
        rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.023,
            width=0.030,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.030, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.011),
        bore=WheelBore(style="round", diameter=0.012),
    )
    tire_mesh = TireGeometry(
        0.085,
        0.045,
        inner_radius=0.055,
        tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.005, radius=0.003),
    )
    for idx, x in enumerate((-0.205, 0.205)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"wheel_{idx}_tire"),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"wheel_{idx}_hub"),
            material=metal,
            name="hub",
        )
        model.articulation(
            f"frame_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, 0.043, 0.095)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=0.320),
    )
    model.articulation(
        "frame_to_toe_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe,
        origin=Origin(xyz=(0.0, -0.055, 0.115)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.5708),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    toe = object_model.get_part("toe_plate")
    handle_slide = object_model.get_articulation("frame_to_handle")
    toe_hinge = object_model.get_articulation("frame_to_toe_plate")

    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            handle,
            elem_a=f"sleeve_{idx}",
            elem_b=f"rail_{idx}",
            reason="The telescoping rail is intentionally represented as a retained member sliding inside its side sleeve.",
        )
        ctx.expect_within(
            handle,
            frame,
            axes="xy",
            inner_elem=f"rail_{idx}",
            outer_elem=f"sleeve_{idx}",
            margin=0.004,
            name=f"handle rail {idx} nests inside side sleeve",
        )
        ctx.expect_overlap(
            handle,
            frame,
            axes="z",
            elem_a=f"rail_{idx}",
            elem_b=f"sleeve_{idx}",
            min_overlap=0.18,
            name=f"collapsed rail {idx} remains inserted",
        )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.320}):
        extended_handle_pos = ctx.part_world_position(handle)
        for idx in (0, 1):
            ctx.expect_within(
                handle,
                frame,
                axes="xy",
                inner_elem=f"rail_{idx}",
                outer_elem=f"sleeve_{idx}",
                margin=0.004,
                name=f"extended rail {idx} stays guided",
            )
            ctx.expect_overlap(
                handle,
                frame,
                axes="z",
                elem_a=f"rail_{idx}",
                elem_b=f"sleeve_{idx}",
                min_overlap=0.08,
                name=f"extended rail {idx} retains insertion",
            )

    ctx.check(
        "handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.30,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.expect_origin_gap(
        frame,
        toe,
        axis="y",
        min_gap=0.045,
        name="deployed toe plate projects forward from the frame",
    )
    ctx.expect_overlap(
        toe,
        frame,
        axes="x",
        min_overlap=0.28,
        elem_a="deck",
        name="toe plate spans nearly the full truck width",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            toe,
            elem_a=f"hinge_cheek_{idx}",
            elem_b="hinge_barrel",
            reason="The toe-plate hinge barrel is intentionally captured between the side cheek brackets.",
        )
        ctx.expect_overlap(
            frame,
            toe,
            axes="x",
            elem_a=f"hinge_cheek_{idx}",
            elem_b="hinge_barrel",
            min_overlap=0.002,
            name=f"toe hinge barrel is captured by cheek {idx}",
        )
        ctx.expect_overlap(
            frame,
            toe,
            axes="yz",
            elem_a=f"hinge_cheek_{idx}",
            elem_b="hinge_barrel",
            min_overlap=0.010,
            name=f"toe hinge cheek {idx} surrounds the barrel axis",
        )

    rest_toe_aabb = ctx.part_world_aabb(toe)
    with ctx.pose({toe_hinge: 1.5708}):
        folded_toe_aabb = ctx.part_world_aabb(toe)
    ctx.check(
        "toe plate folds upward on its hinge",
        rest_toe_aabb is not None
        and folded_toe_aabb is not None
        and folded_toe_aabb[1][2] > rest_toe_aabb[1][2] + 0.12,
        details=f"rest={rest_toe_aabb}, folded={folded_toe_aabb}",
    )

    wheel_joints = [
        object_model.get_articulation("frame_to_wheel_0"),
        object_model.get_articulation("frame_to_wheel_1"),
    ]
    for idx in (0, 1):
        wheel = object_model.get_part(f"wheel_{idx}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle_tube",
            elem_b="hub",
            reason="The live axle is intentionally captured inside the wheel hub bore.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="x",
            elem_a="axle_tube",
            elem_b="hub",
            min_overlap=0.004,
            name=f"wheel {idx} hub is retained on the axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="yz",
            elem_a="axle_tube",
            elem_b="hub",
            min_overlap=0.012,
            name=f"wheel {idx} hub is coaxial with axle",
        )
    ctx.check(
        "both inset wheels have continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=str([j.articulation_type for j in wheel_joints]),
    )

    return ctx.report()


object_model = build_object_model()
