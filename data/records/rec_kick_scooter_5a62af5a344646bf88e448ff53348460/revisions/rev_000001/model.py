from __future__ import annotations

from math import pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
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


def _head_tube_mesh() -> object:
    """A hollow bearing tube around the steering column."""
    return mesh_from_cadquery(
        cq.Workplane("XY").circle(0.036).circle(0.016).extrude(0.120),
        "head_tube",
        tolerance=0.0008,
    )


def _rounded_deck_mesh() -> object:
    deck = (
        cq.Workplane("XY")
        .box(0.640, 0.135, 0.030)
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.006)
    )
    return mesh_from_cadquery(deck, "rounded_deck", tolerance=0.0008)


def _curved_brake_fender_mesh() -> object:
    """Thin arched rear fender, modeled in a hinge-line local frame."""
    length = 0.225
    width = 0.082
    thickness = 0.009
    segments = 18
    geom = MeshGeometry()
    # Four vertices per station: top-left, top-right, bottom-left, bottom-right.
    station_ids: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        s = i / segments
        x = -0.010 - length * s
        # Shallow spring arch: low over the tire at the center, slightly raised at ends.
        center_z = -0.004 - 0.008 * sin(pi * s) + 0.006 * s
        top_z = center_z + thickness / 2.0
        bottom_z = center_z - thickness / 2.0
        left_top = geom.add_vertex(x, -width / 2.0, top_z)
        right_top = geom.add_vertex(x, width / 2.0, top_z)
        left_bottom = geom.add_vertex(x, -width / 2.0, bottom_z)
        right_bottom = geom.add_vertex(x, width / 2.0, bottom_z)
        station_ids.append((left_top, right_top, left_bottom, right_bottom))

    for i in range(segments):
        lt0, rt0, lb0, rb0 = station_ids[i]
        lt1, rt1, lb1, rb1 = station_ids[i + 1]
        # top skin
        geom.add_face(lt0, lt1, rt1)
        geom.add_face(lt0, rt1, rt0)
        # bottom skin
        geom.add_face(lb0, rb1, lb1)
        geom.add_face(lb0, rb0, rb1)
        # left and right edges
        geom.add_face(lt0, lb1, lt1)
        geom.add_face(lt0, lb0, lb1)
        geom.add_face(rt0, rt1, rb1)
        geom.add_face(rt0, rb1, rb0)

    # Close front and rear ends.
    for lt, rt, lb, rb in (station_ids[0], station_ids[-1]):
        geom.add_face(lt, rt, rb)
        geom.add_face(lt, rb, lb)

    return mesh_from_geometry(geom, "brake_fender_shell")


def _scooter_tire(name: str) -> object:
    return mesh_from_geometry(
        TireGeometry(
            0.072,
            0.030,
            inner_radius=0.050,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
            tread=TireTread(style="ribbed", depth=0.0025, count=28, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        name,
    )


def _scooter_hub(name: str) -> object:
    return mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.026,
            rim=WheelRim(
                inner_radius=0.032,
                flange_height=0.004,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="flat",
                bolt_pattern=BoltPattern(
                    count=6,
                    circle_diameter=0.026,
                    hole_diameter=0.003,
                ),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rigid_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    grip = model.material("black_grip_tape", rgba=(0.01, 0.01, 0.01, 1.0))
    tire_mat = model.material("smoky_polyurethane", rgba=(0.04, 0.045, 0.05, 1.0))
    blue = model.material("blue_anodized_hub", rgba=(0.05, 0.26, 0.75, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))

    deck_frame = model.part("deck_frame")
    deck_frame.visual(
        _rounded_deck_mesh(),
        origin=Origin(xyz=(0.020, 0.0, 0.130)),
        material=aluminum,
        name="deck_shell",
    )
    deck_frame.visual(
        Box((0.525, 0.105, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.147)),
        material=grip,
        name="grip_tape",
    )
    # Two side rails leave the centerline clear for the steering column.
    for idx, y in enumerate((-0.036, 0.036)):
        deck_frame.visual(
            Box((0.035, 0.014, 0.070)),
            origin=Origin(xyz=(0.300, y, 0.166)),
            material=aluminum,
            name=f"front_rail_{idx}",
        )
    deck_frame.visual(
        _head_tube_mesh(),
        origin=Origin(xyz=(0.330, 0.0, 0.180)),
        material=aluminum,
        name="head_tube",
    )
    # Rear dropouts carry the rear axle just behind the tail of the deck.
    for idx, y in enumerate((-0.047, 0.047)):
        deck_frame.visual(
            Box((0.140, 0.012, 0.056)),
            origin=Origin(xyz=(-0.365, y, 0.095)),
            material=aluminum,
            name=f"rear_dropout_{idx}",
        )
    deck_frame.visual(
        Cylinder(radius=0.0085, length=0.112),
        origin=Origin(xyz=(-0.390, 0.0, 0.075), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )
    # Brake hinge tabs sit outside the moving fender barrel.
    for idx, y in enumerate((-0.055, 0.055)):
        deck_frame.visual(
            Box((0.030, 0.012, 0.046)),
            origin=Origin(xyz=(-0.286, y, 0.156)),
            material=dark_metal,
            name=f"brake_hinge_tab_{idx}",
        )
    deck_frame.visual(
        Cylinder(radius=0.006, length=0.128),
        origin=Origin(xyz=(-0.286, 0.0, 0.174), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="brake_hinge_pin",
    )

    front_steerer = model.part("front_steerer")
    front_steerer.visual(
        Cylinder(radius=0.017, length=0.820),
        origin=Origin(xyz=(0.0, 0.0, 0.327)),
        material=dark_metal,
        name="steering_column",
    )
    front_steerer.visual(
        Cylinder(radius=0.015, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.705), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar",
    )
    for idx, y in enumerate((-0.205, 0.205)):
        front_steerer.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.705), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"hand_grip_{idx}",
        )
    front_steerer.visual(
        Box((0.110, 0.078, 0.020)),
        origin=Origin(xyz=(0.060, 0.0, -0.076)),
        material=aluminum,
        name="fork_crown",
    )
    for idx, y in enumerate((-0.034, 0.034)):
        front_steerer.visual(
            Cylinder(radius=0.007, length=0.116),
            origin=Origin(xyz=(0.105, y, -0.123)),
            material=aluminum,
            name=f"fork_leg_{idx}",
        )
    front_steerer.visual(
        Cylinder(radius=0.0085, length=0.096),
        origin=Origin(xyz=(0.105, 0.0, -0.165), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    wheel_visual_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    front_wheel.visual(
        _scooter_tire("front_tire_mesh"),
        origin=wheel_visual_origin,
        material=tire_mat,
        name="tire",
    )
    front_wheel.visual(
        _scooter_hub("front_hub_mesh"),
        origin=wheel_visual_origin,
        material=blue,
        name="hub",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        _scooter_tire("rear_tire_mesh"),
        origin=wheel_visual_origin,
        material=tire_mat,
        name="tire",
    )
    rear_wheel.visual(
        _scooter_hub("rear_hub_mesh"),
        origin=wheel_visual_origin,
        material=blue,
        name="hub",
    )

    brake_fender = model.part("brake_fender")
    brake_fender.visual(
        _curved_brake_fender_mesh(),
        material=dark_metal,
        name="fender_shell",
    )
    brake_fender.visual(
        Cylinder(radius=0.011, length=0.076),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    brake_fender.visual(
        Box((0.118, 0.060, 0.005)),
        origin=Origin(xyz=(-0.115, 0.0, -0.002)),
        material=rubber,
        name="foot_pad",
    )
    brake_fender.visual(
        Box((0.058, 0.045, 0.006)),
        origin=Origin(xyz=(-0.130, 0.0, -0.014)),
        material=rubber,
        name="brake_pad_under",
    )

    model.articulation(
        "deck_to_steerer",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=front_steerer,
        origin=Origin(xyz=(0.330, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "steerer_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_steerer,
        child=front_wheel,
        origin=Origin(xyz=(0.105, 0.0, -0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck_frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.390, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "deck_to_brake_fender",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=brake_fender,
        origin=Origin(xyz=(-0.286, 0.0, 0.174)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_frame")
    steerer = object_model.get_part("front_steerer")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    brake = object_model.get_part("brake_fender")
    steer_joint = object_model.get_articulation("deck_to_steerer")
    brake_joint = object_model.get_articulation("deck_to_brake_fender")

    ctx.allow_overlap(
        steerer,
        front_wheel,
        elem_a="front_axle",
        elem_b="hub",
        reason="The stationary front axle is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="hub",
        reason="The rear axle is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        deck,
        brake,
        elem_a="brake_hinge_pin",
        elem_b="hinge_barrel",
        reason="The fixed hinge pin is intentionally seated inside the rotating brake-fender barrel.",
    )
    ctx.allow_overlap(
        deck,
        steerer,
        elem_a="head_tube",
        elem_b="steering_column",
        reason="The steering stem is intentionally captured in the head-tube bearing sleeve.",
    )

    ctx.expect_within(
        steerer,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="hub",
        margin=0.004,
        name="front axle is centered through hub",
    )
    ctx.expect_overlap(
        steerer,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="hub",
        min_overlap=0.022,
        name="front axle spans the hub",
    )
    ctx.expect_within(
        deck,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="hub",
        margin=0.004,
        name="rear axle is centered through hub",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="hub",
        min_overlap=0.022,
        name="rear axle spans the hub",
    )
    ctx.expect_within(
        deck,
        brake,
        axes="xz",
        inner_elem="brake_hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.003,
        name="brake hinge pin is captured in barrel",
    )
    ctx.expect_overlap(
        deck,
        brake,
        axes="y",
        elem_a="brake_hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.060,
        name="brake hinge pin runs through barrel",
    )
    ctx.expect_within(
        steerer,
        deck,
        axes="xy",
        inner_elem="steering_column",
        outer_elem="head_tube",
        margin=0.002,
        name="steering column is centered in head tube",
    )
    ctx.expect_overlap(
        steerer,
        deck,
        axes="z",
        elem_a="steering_column",
        elem_b="head_tube",
        min_overlap=0.090,
        name="steering column passes through head tube",
    )

    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        positive_elem="tire",
        negative_elem="deck_shell",
        min_gap=0.012,
        name="front wheel clears deck nose",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="tire",
        min_gap=0.010,
        name="rear wheel sits just behind deck tail",
    )
    ctx.expect_gap(
        brake,
        rear_wheel,
        axis="z",
        positive_elem="brake_pad_under",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.030,
        name="brake pad rests just above rear tire",
    )

    wheel_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steer_joint: 0.50}):
        wheel_yawed = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws about steering column",
        wheel_rest is not None
        and wheel_yawed is not None
        and wheel_yawed[1] > wheel_rest[1] + 0.035,
        details=f"rest={wheel_rest}, yawed={wheel_yawed}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(brake, elem="brake_pad_under")
    with ctx.pose({brake_joint: 0.24}):
        pressed_pad_aabb = ctx.part_element_world_aabb(brake, elem="brake_pad_under")
    ctx.check(
        "rear fender brake rotates downward",
        rest_pad_aabb is not None
        and pressed_pad_aabb is not None
        and pressed_pad_aabb[0][2] < rest_pad_aabb[0][2] - 0.020,
        details=f"rest={rest_pad_aabb}, pressed={pressed_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
