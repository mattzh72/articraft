from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder whose local +Z axis spans start -> end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder {name}")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="appliance_hand_truck")

    powder_blue = model.material("powder_blue", rgba=(0.05, 0.23, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    worn_plate = model.material("worn_nose_plate", rgba=(0.36, 0.38, 0.37, 1.0))

    frame = model.part("frame")

    # Tall welded tubular back frame, sized like a real appliance dolly.
    for y, suffix in [(-0.215, "0"), (0.215, "1")]:
        _cylinder_between(
            frame,
            (0.0, y, 0.10),
            (0.0, y, 1.42),
            0.018,
            material=powder_blue,
            name=f"upright_tube_{suffix}",
        )
        _cylinder_between(
            frame,
            (0.0, y, 1.41),
            (-0.035, y, 1.52),
            0.018,
            material=powder_blue,
            name=f"handle_post_{suffix}",
        )
        _cylinder_between(
            frame,
            (0.0, y, 0.18),
            (-0.125, y, 0.18),
            0.016,
            material=powder_blue,
            name=f"axle_boss_{suffix}",
        )
        _cylinder_between(
            frame,
            (0.0, y, 0.11),
            (-0.125, y, 0.18),
            0.013,
            material=powder_blue,
            name=f"lower_strut_{suffix}",
        )

    for z, label, radius in [
        (1.40, "top_crossbar", 0.018),
        (0.92, "upper_crossbar", 0.014),
        (0.56, "middle_crossbar", 0.014),
        (0.22, "lower_crossbar", 0.016),
    ]:
        _cylinder_between(
            frame,
            (0.0, -0.245, z),
            (0.0, 0.245, z),
            radius,
            material=powder_blue,
            name=label,
        )

    _cylinder_between(
        frame,
        (-0.035, -0.255, 1.52),
        (-0.035, 0.255, 1.52),
        0.021,
        material=black_rubber,
        name="handle_grip",
    )

    # Rear axle tube stops at the inside faces of the wheels, with visible bosses
    # and bracing back to the frame uprights.
    frame.visual(
        Cylinder(radius=0.014, length=0.740),
        origin=Origin(xyz=(-0.125, 0.0, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )

    # Transverse hinge pin for the folding toe plate. The moving knuckles are
    # authored on the toe-plate link and rotate around this same line.
    frame.visual(
        Cylinder(radius=0.009, length=0.530),
        origin=Origin(xyz=(0.025, 0.0, 0.070), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="toe_hinge_pin",
    )
    frame.visual(
        Box((0.040, 0.040, 0.055)),
        origin=Origin(xyz=(0.012, -0.245, 0.092)),
        material=dark_steel,
        name="hinge_tab_0",
    )
    frame.visual(
        Box((0.040, 0.040, 0.055)),
        origin=Origin(xyz=(0.012, 0.245, 0.092)),
        material=dark_steel,
        name="hinge_tab_1",
    )

    # Rubber appliance pads on the back frame keep the object recognizably an
    # appliance hand truck rather than a generic luggage dolly.
    frame.visual(
        Box((0.026, 0.455, 0.055)),
        origin=Origin(xyz=(0.020, 0.0, 0.67)),
        material=black_rubber,
        name="middle_appliance_pad",
    )
    frame.visual(
        Box((0.026, 0.420, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 1.08)),
        material=black_rubber,
        name="upper_appliance_pad",
    )

    # Large pneumatic-style rear wheels with separate tire and metal hub visuals.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.180,
            0.080,
            inner_radius=0.128,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.07),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "large_utility_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.130,
            0.070,
            rim=WheelRim(
                inner_radius=0.082,
                flange_height=0.010,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.034,
                width=0.054,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.045,
                    hole_diameter=0.005,
                ),
            ),
            face=WheelFace(dish_depth=0.009, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.026),
        ),
        "stamped_utility_wheel",
    )

    for y, suffix in [(-0.330, "0"), (0.330, "1")]:
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        wheel.visual(wheel_mesh, material=galvanized, name="hub")
        model.articulation(
            f"frame_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.125, y, 0.180), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=10.0),
        )

    toe_plate = model.part("toe_plate")

    # Broad folding nose plate. Its part frame is the transverse hinge line; in
    # the carrying pose it projects forward near the floor, then folds upward.
    toe_plate.visual(
        Box((0.430, 0.480, 0.018)),
        origin=Origin(xyz=(0.215, 0.0, -0.037)),
        material=worn_plate,
        name="nose_plate",
    )
    toe_plate.visual(
        Box((0.024, 0.480, 0.070)),
        origin=Origin(xyz=(0.421, 0.0, -0.006)),
        material=worn_plate,
        name="front_lip",
    )
    for y, suffix in [(-0.155, "0"), (0.0, "1"), (0.155, "2")]:
        toe_plate.visual(
            Box((0.355, 0.030, 0.018)),
            origin=Origin(xyz=(0.230, y, -0.019)),
            material=galvanized,
            name=f"raised_rib_{suffix}",
        )
        toe_plate.visual(
            Box((0.060, 0.030, 0.020)),
            origin=Origin(xyz=(0.030, y, -0.022)),
            material=worn_plate,
            name=f"hinge_leaf_{suffix}",
        )
        _cylinder_between(
            toe_plate,
            (0.0, y - 0.050, 0.0),
            (0.0, y + 0.050, 0.0),
            0.018,
            material=worn_plate,
            name=f"hinge_sleeve_{suffix}",
        )

    model.articulation(
        "frame_to_toe_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_plate,
        origin=Origin(xyz=(0.025, 0.0, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    toe_plate = object_model.get_part("toe_plate")
    toe_hinge = object_model.get_articulation("frame_to_toe_plate")

    for suffix in ("0", "1", "2"):
        ctx.allow_overlap(
            frame,
            toe_plate,
            elem_a="toe_hinge_pin",
            elem_b=f"hinge_sleeve_{suffix}",
            reason="The hinge sleeve is intentionally represented as a captured knuckle around the transverse hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            toe_plate,
            axes="y",
            elem_a="toe_hinge_pin",
            elem_b=f"hinge_sleeve_{suffix}",
            min_overlap=0.080,
            name=f"hinge sleeve {suffix} wraps the pin along its length",
        )

    for suffix in ("0", "1"):
        wheel = object_model.get_part(f"wheel_{suffix}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="hub",
            reason="The large wheel hub is intentionally captured around the steel axle so the wheel can spin on it.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="hub",
            min_overlap=0.050,
            name=f"wheel {suffix} hub is retained on the axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xz",
            elem_a="wheel_axle",
            elem_b="hub",
            min_overlap=0.020,
            name=f"wheel {suffix} hub is concentric with the axle",
        )

    ctx.check(
        "toe plate has realistic folding limit",
        toe_hinge.motion_limits is not None
        and toe_hinge.motion_limits.lower == 0.0
        and abs(toe_hinge.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={toe_hinge.motion_limits}",
    )

    wheel_types_ok = all(
        object_model.get_articulation(f"frame_to_wheel_{suffix}").articulation_type
        == ArticulationType.CONTINUOUS
        for suffix in ("0", "1")
    )
    ctx.check("both large wheels spin continuously", wheel_types_ok)

    with ctx.pose({toe_hinge: 0.0}):
        carry_aabb = ctx.part_world_aabb(toe_plate)
        ctx.check(
            "toe plate projects forward for loading",
            carry_aabb is not None and carry_aabb[1][0] > 0.42 and carry_aabb[0][2] < 0.04,
            details=f"aabb={carry_aabb}",
        )

    with ctx.pose({toe_hinge: math.pi / 2.0}):
        folded_aabb = ctx.part_world_aabb(toe_plate)
        ctx.check(
            "toe plate folds upward against the frame",
            folded_aabb is not None and folded_aabb[1][2] > 0.48 and folded_aabb[1][0] < 0.10,
            details=f"aabb={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
