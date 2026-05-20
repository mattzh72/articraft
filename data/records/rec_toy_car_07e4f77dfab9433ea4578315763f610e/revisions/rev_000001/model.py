from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


PI = math.pi


def _coupe_body_mesh():
    """One-piece toy coupe shell with short hood, cabin, trunk, and wheel arches."""
    width = 0.064
    side_profile = [
        (-0.078, 0.020),
        (0.078, 0.020),
        (0.078, 0.031),
        (0.052, 0.039),  # short hood/nose
        (0.020, 0.043),
        (0.004, 0.058),  # windshield/A-pillar rise
        (-0.028, 0.061),
        (-0.052, 0.046),  # rear glass into trunk
        (-0.078, 0.038),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(width * 0.5, both=True)
    )
    # Open die-cast-style wheel arches expose the four wheels from the sides
    # while leaving a central floor spine to support the axles and steering pin.
    for wheel_x in (-0.052, 0.052):
        for side_y in (0.016, -0.036):
            arch_cutter = (
                cq.Workplane("XZ")
                .center(wheel_x, 0.019)
                .circle(0.0185)
                .extrude(0.020)
                .translate((0.0, side_y, 0.0))
            )
            body = body.cut(arch_cutter)
    return body


def _side_glass_mesh(sign: float):
    """Trapezoid side window panel for one side of the coupe."""
    pts = [
        (-0.033, 0.043),
        (-0.016, 0.057),
        (0.010, 0.056),
        (0.020, 0.044),
    ]
    glass = cq.Workplane("XZ").polyline(pts).close().extrude(0.0024, both=True)
    return glass.translate((0.0, sign * 0.0320, 0.0))


def _make_wheel_meshes():
    wheel = WheelGeometry(
        0.0105,
        0.0085,
        rim=WheelRim(
            inner_radius=0.0068,
            flange_height=0.0012,
            flange_thickness=0.0007,
            bead_seat_depth=0.0006,
        ),
        hub=WheelHub(
            radius=0.0036,
            width=0.007,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=0.0052,
                hole_diameter=0.0008,
            ),
        ),
        face=WheelFace(dish_depth=0.0012, front_inset=0.0005, rear_inset=0.0005),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.00055, window_radius=0.0018),
        bore=WheelBore(style="round", diameter=0.0055),
    )
    tire = TireGeometry(
        0.015,
        0.010,
        inner_radius=0.0106,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.0012, count=18, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.0012, depth=0.0006),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.0012, radius=0.0008),
    )
    return (
        mesh_from_geometry(wheel, "silver_spoked_wheel"),
        mesh_from_geometry(tire, "black_toy_tire"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="die_cast_toy_coupe")

    red = model.material("gloss_red", rgba=(0.78, 0.03, 0.02, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    glass = model.material("smoky_glass", rgba=(0.02, 0.035, 0.045, 0.92))
    chrome = model.material("toy_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    dark_metal = model.material("dark_steel", rgba=(0.08, 0.08, 0.075, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_coupe_body_mesh(), "coupe_body_shell", tolerance=0.0005),
        material=red,
        name="body_shell",
    )
    # Underside axles and the front steering socket are fixed to the body, so
    # the wheels have a visible mounting path rather than hovering beside it.
    body.visual(
        Cylinder(radius=0.00275, length=0.076),
        origin=Origin(xyz=(-0.052, 0.0, 0.015), rpy=(PI / 2, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle_pin",
    )
    for side_name, y in (("0", 0.0331), ("1", -0.0331)):
        body.visual(
            Cylinder(radius=0.0062, length=0.0010),
            origin=Origin(xyz=(-0.052, y, 0.015), rpy=(PI / 2, 0.0, 0.0)),
            material=chrome,
            name=f"rear_washer_{side_name}",
        )
    for side_name, y in (("0", 0.018), ("1", -0.018)):
        body.visual(
            Box((0.012, 0.005, 0.010)),
            origin=Origin(xyz=(-0.052, y, 0.018)),
            material=dark_metal,
            name=f"rear_axle_saddle_{side_name}",
        )
    body.visual(
        Cylinder(radius=0.006, length=0.007),
        origin=Origin(xyz=(0.052, 0.0, 0.021), rpy=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="steering_socket",
    )
    body.visual(
        Box((0.0018, 0.054, 0.0025)),
        origin=Origin(xyz=(-0.051, 0.0, 0.046)),
        material=black,
        name="trunk_seam",
    )
    body.visual(
        Box((0.0015, 0.050, 0.0020)),
        origin=Origin(xyz=(0.039, 0.0, 0.041)),
        material=black,
        name="hood_shut_line",
    )
    for side, sign in (("side_0", 1.0), ("side_1", -1.0)):
        y = sign * 0.0322
        body.visual(
            Box((0.062, 0.0020, 0.0018)),
            origin=Origin(xyz=(-0.006, y, 0.030)),
            material=black,
            name=f"{side}_lower_door_cut",
        )
        body.visual(
            Box((0.0014, 0.0020, 0.026)),
            origin=Origin(xyz=(0.021, y, 0.041)),
            material=black,
            name=f"{side}_a_pillar_cut",
        )
        body.visual(
            Box((0.0014, 0.0020, 0.023)),
            origin=Origin(xyz=(-0.039, y, 0.041)),
            material=black,
            name=f"{side}_b_pillar_cut",
        )
        body.visual(
            mesh_from_cadquery(_side_glass_mesh(sign), f"{side}_side_glass", tolerance=0.0004),
            material=glass,
            name=f"{side}_side_glass",
        )

    front_axle = model.part("front_axle")
    front_axle.visual(
        Cylinder(radius=0.00275, length=0.076),
        origin=Origin(rpy=(PI / 2, 0.0, 0.0)),
        material=dark_metal,
        name="crossbar",
    )
    front_axle.visual(
        Cylinder(radius=0.0028, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="steering_post",
    )
    front_axle.visual(
        Box((0.016, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=dark_metal,
        name="center_yoke",
    )
    for side_name, y in (("0", 0.0331), ("1", -0.0331)):
        front_axle.visual(
            Cylinder(radius=0.0062, length=0.0010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(PI / 2, 0.0, 0.0)),
            material=chrome,
            name=f"front_washer_{side_name}",
        )

    model.articulation(
        "body_to_front_axle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_axle,
        origin=Origin(xyz=(0.052, 0.0, 0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-0.45, upper=0.45),
    )

    wheel_mesh, tire_mesh = _make_wheel_meshes()
    wheel_specs = [
        ("front_wheel_0", front_axle, (0.0, 0.038, 0.0)),
        ("front_wheel_1", front_axle, (0.0, -0.038, 0.0)),
        ("rear_wheel_0", body, (-0.052, 0.038, 0.015)),
        ("rear_wheel_1", body, (-0.052, -0.038, 0.015)),
    ]
    for wheel_name, parent, xyz in wheel_specs:
        wheel_part = model.part(wheel_name)
        # The wheel helper spins about local X; this visual yaw turns local X
        # into the car's lateral axle direction.
        wheel_part.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, PI / 2)),
            material=black,
            name="tire",
        )
        wheel_part.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, PI / 2)),
            material=chrome,
            name="rim",
        )
        model.articulation(
            f"{parent.name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel_part,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=20.0),
        )

    for door_name, sign, axis_z in (("door_0", 1.0, -1.0), ("door_1", -1.0, 1.0)):
        door = model.part(door_name)
        outward = sign * 0.0015
        door.visual(
            Box((0.054, 0.0020, 0.023)),
            origin=Origin(xyz=(-0.027, outward, -0.001)),
            material=red,
            name="outer_panel",
        )
        door.visual(
            Box((0.037, 0.0014, 0.011)),
            origin=Origin(xyz=(-0.017, outward * 1.05, 0.014)),
            material=glass,
            name="window",
        )
        door.visual(
            Cylinder(radius=0.0010, length=0.025),
            origin=Origin(xyz=(0.0, -sign * 0.0002, 0.000), rpy=(0.0, 0.0, 0.0)),
            material=chrome,
            name="hinge_barrel",
        )
        door.visual(
            Box((0.008, 0.0014, 0.0020)),
            origin=Origin(xyz=(-0.037, sign * 0.0026, 0.002)),
            material=chrome,
            name="pull_handle",
        )
        # Black perimeter strips make the door cut read clearly at toy scale.
        door.visual(
            Box((0.055, 0.0010, 0.0012)),
            origin=Origin(xyz=(-0.027, sign * 0.0022, -0.013)),
            material=black,
            name="lower_cut",
        )
        door.visual(
            Box((0.0011, 0.0010, 0.025)),
            origin=Origin(xyz=(-0.054, sign * 0.0022, -0.001)),
            material=black,
            name="rear_cut",
        )
        door.visual(
            Box((0.055, 0.0010, 0.0012)),
            origin=Origin(xyz=(-0.027, sign * 0.0022, 0.011)),
            material=black,
            name="belt_cut",
        )
        model.articulation(
            f"body_to_{door_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(0.021, sign * 0.0332, 0.042)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=0.35, velocity=2.2, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_axle = object_model.get_part("front_axle")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    steer = object_model.get_articulation("body_to_front_axle")
    door_0_hinge = object_model.get_articulation("body_to_door_0")
    door_1_hinge = object_model.get_articulation("body_to_door_1")

    ctx.expect_contact(
        front_axle,
        body,
        elem_a="steering_post",
        elem_b="steering_socket",
        contact_tol=0.001,
        name="front steering post is supported by body socket",
    )
    ctx.expect_gap(
        door_0,
        body,
        axis="y",
        positive_elem="outer_panel",
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.004,
        name="door_0 closes just outside side body",
    )
    ctx.expect_gap(
        body,
        door_1,
        axis="y",
        positive_elem="body_shell",
        negative_elem="outer_panel",
        min_gap=0.0002,
        max_gap=0.004,
        name="door_1 closes just outside side body",
    )

    closed_aabb_0 = ctx.part_world_aabb(door_0)
    closed_aabb_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({door_0_hinge: 0.75, door_1_hinge: 0.75}):
        open_aabb_0 = ctx.part_world_aabb(door_0)
        open_aabb_1 = ctx.part_world_aabb(door_1)
    closed_0_y = None if closed_aabb_0 is None else (closed_aabb_0[0][1] + closed_aabb_0[1][1]) * 0.5
    closed_1_y = None if closed_aabb_1 is None else (closed_aabb_1[0][1] + closed_aabb_1[1][1]) * 0.5
    open_0_y = None if open_aabb_0 is None else (open_aabb_0[0][1] + open_aabb_0[1][1]) * 0.5
    open_1_y = None if open_aabb_1 is None else (open_aabb_1[0][1] + open_aabb_1[1][1]) * 0.5
    ctx.check(
        "doors rotate outward about vertical A-pillar hinges",
        closed_0_y is not None
        and closed_1_y is not None
        and open_0_y is not None
        and open_1_y is not None
        and open_0_y > closed_0_y + 0.004
        and open_1_y < closed_1_y - 0.004,
        details=f"closed_0_y={closed_0_y}, open_0_y={open_0_y}, closed_1_y={closed_1_y}, open_1_y={open_1_y}",
    )

    straight_pos = ctx.part_world_position(front_wheel_0)
    with ctx.pose({steer: 0.35}):
        steered_pos = ctx.part_world_position(front_wheel_0)
    ctx.check(
        "front axle yaws the front wheel pair under the nose",
        straight_pos is not None
        and steered_pos is not None
        and abs(steered_pos[0] - straight_pos[0]) > 0.004,
        details=f"straight={straight_pos}, steered={steered_pos}",
    )

    return ctx.report()


object_model = build_object_model()
