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


def _tube_between(part, name, start, end, radius, material):
    """Add a round tube between two points in a part frame.

    The scooter frame is laid out in X/Z side view, so all slanted frame tubes
    in this script lie in an XZ plane and can use a single pitch rotation.
    """

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1.0e-9:
        # All transverse axles/handlebars are authored separately with a fixed
        # roll; keep this helper for the fore-aft structural tubes.
        raise ValueError("tube helper expects an XZ-plane tube")
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _axial_cylinder(part, name, *, xyz, radius, length, axis, material):
    """Add a primitive cylinder along one cardinal axis."""

    if axis == "y":
        rpy = (-math.pi / 2.0, 0.0, 0.0)
    elif axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif axis == "z":
        rpy = (0.0, 0.0, 0.0)
    else:
        raise ValueError(axis)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="off_road_folding_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.045, 0.048, 0.052, 1.0))
    matte_black = model.material("matte_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    grip = model.material("coarse_black_grip", rgba=(0.015, 0.014, 0.012, 1.0))
    safety_orange = model.material("orange_frame_accent", rgba=(0.95, 0.34, 0.05, 1.0))
    stanchion = model.material("polished_fork_stanchion", rgba=(0.82, 0.84, 0.82, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.124,
            0.082,
            rim=WheelRim(
                inner_radius=0.078,
                flange_height=0.010,
                flange_thickness=0.005,
                bead_seat_depth=0.005,
            ),
            hub=WheelHub(
                radius=0.034,
                width=0.070,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=6,
                    circle_diameter=0.048,
                    hole_diameter=0.005,
                ),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.005, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.028),
        ),
        "wide_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.180,
            0.096,
            inner_radius=0.126,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.10),
            tread=TireTread(style="block", depth=0.011, count=24, land_ratio=0.55),
            grooves=(
                TireGroove(center_offset=-0.020, width=0.006, depth=0.004),
                TireGroove(center_offset=0.020, width=0.006, depth=0.004),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.07),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "knobby_pneumatic_tire",
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.76, 0.205, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, 0.180)),
        material=safety_orange,
        name="deck_shell",
    )
    deck.visual(
        Box((0.64, 0.155, 0.008)),
        origin=Origin(xyz=(0.060, 0.0, 0.212)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        Box((0.120, 0.028, 0.080)),
        origin=Origin(xyz=(0.410, -0.078, 0.232)),
        material=safety_orange,
        name="nose_riser_0",
    )
    deck.visual(
        Box((0.120, 0.028, 0.080)),
        origin=Origin(xyz=(0.410, 0.078, 0.232)),
        material=safety_orange,
        name="nose_riser_1",
    )
    deck.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.430, 0.0, 0.2125)),
        material=dark_metal,
        name="steering_bearing_cup",
    )
    _tube_between(deck, "nose_strut_0", (0.365, -0.052, 0.235), (0.405, -0.052, 0.262), 0.018, aluminum)
    _tube_between(deck, "nose_strut_1", (0.365, 0.052, 0.235), (0.405, 0.052, 0.262), 0.018, aluminum)
    _tube_between(deck, "rear_stay_0", (-0.285, -0.082, 0.187), (-0.515, -0.082, 0.180), 0.015, aluminum)
    _tube_between(deck, "rear_stay_1", (-0.285, 0.082, 0.187), (-0.515, 0.082, 0.180), 0.015, aluminum)
    _tube_between(deck, "rear_upper_stay_0", (-0.250, -0.083, 0.212), (-0.515, -0.083, 0.180), 0.010, aluminum)
    _tube_between(deck, "rear_upper_stay_1", (-0.250, 0.083, 0.212), (-0.515, 0.083, 0.180), 0.010, aluminum)
    deck.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(-0.515, 0.0, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )

    front_yoke = model.part("front_yoke")
    # Local frame is on the steering/folding hinge block above the deck nose.
    front_yoke.visual(
        Box((0.120, 0.175, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, -0.055)),
        material=dark_metal,
        name="hinge_block",
    )
    front_yoke.visual(
        Box((0.052, 0.022, 0.105)),
        origin=Origin(xyz=(0.000, -0.069, 0.012)),
        material=dark_metal,
        name="hinge_cheek_0",
    )
    front_yoke.visual(
        Box((0.052, 0.022, 0.105)),
        origin=Origin(xyz=(0.000, 0.069, 0.012)),
        material=dark_metal,
        name="hinge_cheek_1",
    )
    _axial_cylinder(
        front_yoke,
        "pin_cap_0",
        xyz=(0.0, -0.087, 0.0),
        radius=0.018,
        length=0.010,
        axis="y",
        material=aluminum,
    )
    _axial_cylinder(
        front_yoke,
        "pin_cap_1",
        xyz=(0.0, 0.087, 0.0),
        radius=0.018,
        length=0.010,
        axis="y",
        material=aluminum,
    )
    front_yoke.visual(
        Cylinder(radius=0.024, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=aluminum,
        name="steerer_tube",
    )
    front_yoke.visual(
        Box((0.150, 0.220, 0.042)),
        origin=Origin(xyz=(0.105, 0.0, 0.032)),
        material=dark_metal,
        name="fork_crown",
    )
    front_yoke.visual(
        Box((0.090, 0.030, 0.080)),
        origin=Origin(xyz=(0.035, -0.095, -0.002)),
        material=dark_metal,
        name="side_gusset_0",
    )
    front_yoke.visual(
        Box((0.090, 0.030, 0.080)),
        origin=Origin(xyz=(0.035, 0.095, -0.002)),
        material=dark_metal,
        name="side_gusset_1",
    )
    _tube_between(front_yoke, "fork_stanchion_0", (0.095, -0.076, 0.020), (0.235, -0.076, -0.180), 0.015, stanchion)
    _tube_between(front_yoke, "fork_stanchion_1", (0.095, 0.076, 0.020), (0.235, 0.076, -0.180), 0.015, stanchion)
    _tube_between(front_yoke, "fork_slider_0", (0.155, -0.077, -0.070), (0.235, -0.077, -0.180), 0.021, dark_metal)
    _tube_between(front_yoke, "fork_slider_1", (0.155, 0.077, -0.070), (0.235, 0.077, -0.180), 0.021, dark_metal)
    front_yoke.visual(
        Box((0.100, 0.205, 0.030)),
        origin=Origin(xyz=(0.155, 0.0, 0.030)),
        material=dark_metal,
        name="fork_bridge",
    )
    front_yoke.visual(
        Cylinder(radius=0.014, length=0.232),
        origin=Origin(xyz=(0.235, 0.0, -0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.026, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fold_barrel",
    )
    _axial_cylinder(
        stem,
        "lower_lug",
        xyz=(0.0, 0.0, 0.050),
        radius=0.024,
        length=0.090,
        axis="z",
        material=dark_metal,
    )
    _tube_between(stem, "steering_column", (0.0, 0.0, 0.035), (-0.070, 0.0, 0.900), 0.022, aluminum)
    _tube_between(stem, "front_brace", (0.028, 0.0, 0.075), (-0.047, 0.0, 0.520), 0.010, dark_metal)
    _axial_cylinder(
        stem,
        "handlebar",
        xyz=(-0.073, 0.0, 0.900),
        radius=0.018,
        length=0.610,
        axis="y",
        material=dark_metal,
    )
    _axial_cylinder(
        stem,
        "grip_0",
        xyz=(-0.073, -0.260, 0.900),
        radius=0.024,
        length=0.115,
        axis="y",
        material=matte_black,
    )
    _axial_cylinder(
        stem,
        "grip_1",
        xyz=(-0.073, 0.260, 0.900),
        radius=0.024,
        length=0.115,
        axis="y",
        material=matte_black,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=matte_black,
        name="rear_tire",
    )
    rear_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rear_rim",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=matte_black,
        name="front_tire",
    )
    front_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="front_rim",
    )

    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.515, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_yoke,
        origin=Origin(xyz=(0.430, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.78, upper=0.78),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=front_yoke,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # Positive rotation folds the upright stem rearward/down toward the deck.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.42),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_yoke,
        child=front_wheel,
        origin=Origin(xyz=(0.235, 0.0, -0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_yoke = object_model.get_part("front_yoke")
    stem = object_model.get_part("stem")
    rear_wheel = object_model.get_part("rear_wheel")
    front_wheel = object_model.get_part("front_wheel")
    steering = object_model.get_articulation("steering_yaw")
    fold = object_model.get_articulation("stem_fold")
    rear_spin = object_model.get_articulation("rear_wheel_spin")
    front_spin = object_model.get_articulation("front_wheel_spin")

    ctx.check(
        "primary scooter articulations are present",
        all(j is not None for j in (steering, fold, rear_spin, front_spin)),
        details="Scooter needs steering yaw, stem fold, and two wheel spin joints.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_rim",
        reason="The rear axle is intentionally captured inside the wheel hub bore so the pneumatic wheel remains supported while spinning.",
    )
    ctx.allow_overlap(
        front_yoke,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_rim",
        reason="The front axle is intentionally captured inside the hub bore between the suspension fork sliders.",
    )

    # The folding barrel stays captured between the hinge cheeks in every stem
    # angle because the stem frame is exactly on the hinge pin.
    ctx.expect_contact(
        deck,
        front_yoke,
        elem_a="steering_bearing_cup",
        elem_b="steerer_tube",
        contact_tol=0.001,
        name="front assembly is seated in the deck steering bearing",
    )
    ctx.expect_within(
        stem,
        front_yoke,
        axes="y",
        inner_elem="fold_barrel",
        outer_elem="hinge_block",
        margin=0.0,
        name="folding barrel is inside hinge block width",
    )
    ctx.expect_contact(
        stem,
        front_yoke,
        elem_a="fold_barrel",
        elem_b="hinge_cheek_0",
        contact_tol=0.001,
        name="folding barrel is clipped by first hinge cheek",
    )
    ctx.expect_contact(
        stem,
        front_yoke,
        elem_a="fold_barrel",
        elem_b="hinge_cheek_1",
        contact_tol=0.001,
        name="folding barrel is clipped by second hinge cheek",
    )

    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        min_gap=0.020,
        positive_elem="front_tire",
        negative_elem="deck_shell",
        name="front knobby tire clears the deck nose",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        min_gap=0.020,
        positive_elem="deck_shell",
        negative_elem="rear_tire",
        name="rear knobby tire clears the deck tail",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="yz",
        elem_a="rear_axle",
        elem_b="rear_rim",
        min_overlap=0.015,
        name="rear wheel is retained on its axle",
    )
    ctx.expect_overlap(
        front_yoke,
        front_wheel,
        axes="yz",
        elem_a="front_axle",
        elem_b="front_rim",
        min_overlap=0.015,
        name="front wheel is retained in the suspension fork",
    )

    rest_bar = ctx.part_element_world_aabb(stem, elem="handlebar")
    with ctx.pose({fold: 1.42}):
        folded_bar = ctx.part_element_world_aabb(stem, elem="handlebar")
        ctx.expect_within(
            stem,
            front_yoke,
            axes="y",
            inner_elem="fold_barrel",
            outer_elem="hinge_block",
            margin=0.0,
            name="folded stem remains clipped in hinge block width",
        )

    ctx.check(
        "folding stem moves rearward and lower",
        rest_bar is not None
        and folded_bar is not None
        and folded_bar[1][0] < rest_bar[0][0] - 0.45
        and folded_bar[1][2] < rest_bar[1][2] - 0.55,
        details=f"rest_handlebar_aabb={rest_bar}, folded_handlebar_aabb={folded_bar}",
    )

    return ctx.report()


object_model = build_object_model()
