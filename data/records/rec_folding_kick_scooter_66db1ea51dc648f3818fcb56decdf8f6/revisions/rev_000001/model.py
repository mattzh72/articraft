from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    MotionProperties,
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
    rounded_rect_profile,
)


def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extend: float = 0.0,
) -> tuple[Origin, float]:
    """Return an origin/length for a cylinder whose local +Z spans start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("Cylinder span must have positive length")

    ux, uy, uz = vx / length, vy / length, vz / length
    sx -= ux * extend
    sy -= uy * extend
    sz -= uz * extend
    ex += ux * extend
    ey += uy * extend
    ez += uz * extend
    length += 2.0 * extend

    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
    extend: float = 0.002,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end, extend=extend)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_scooter_wheel(part, *, prefix: str, tire_mat, rim_mat, hub_mat) -> None:
    """Small urethane scooter wheel with separate tire, rim, and hub visuals."""
    tire = TireGeometry(
        0.055,
        0.040,
        inner_radius=0.036,
        carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.035),
        tread=TireTread(style="chevron", depth=0.002, count=18, angle_deg=18.0, land_ratio=0.68),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    rim = WheelGeometry(
        0.039,
        0.034,
        rim=WheelRim(
            inner_radius=0.024,
            flange_height=0.003,
            flange_thickness=0.002,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(radius=0.014, width=0.030, cap_style="domed"),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.005),
        bore=WheelBore(style="round", diameter=0.007),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=tire_mat, name="tire")
    part.visual(mesh_from_geometry(rim, f"{prefix}_rim"), material=rim_mat, name="rim")
    part.visual(
        Cylinder(radius=0.016, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_mat,
        name="hub_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_lean_scooter")

    deck_mat = model.material("teal_powder_coated_deck", rgba=(0.02, 0.42, 0.50, 1.0))
    grip_mat = model.material("black_grip_tape", rgba=(0.015, 0.015, 0.014, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_metal_mat = model.material("dark_hinge_metal", rgba=(0.08, 0.09, 0.09, 1.0))
    rubber_mat = model.material("charcoal_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    urethane_mat = model.material("translucent_blue_urethane", rgba=(0.02, 0.24, 0.80, 1.0))
    rim_mat = model.material("white_polymer_rims", rgba=(0.92, 0.93, 0.88, 1.0))

    deck = model.part("deck")
    deck_shell = ExtrudeGeometry(
        rounded_rect_profile(0.640, 0.160, 0.035, corner_segments=10),
        0.035,
        cap=True,
        center=True,
    )
    deck.visual(mesh_from_geometry(deck_shell, "rounded_flat_deck"), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=deck_mat, name="flat_deck")
    deck.visual(Box((0.500, 0.112, 0.004)), origin=Origin(xyz=(-0.010, 0.0, 0.104)), material=grip_mat, name="grip_pad")
    # Deck-front hardware: the folding stem hinge and the lean-steer crown mount
    # are fixed to the deck root; the moving barrels are authored on child parts.
    deck.visual(Box((0.090, 0.198, 0.025)), origin=Origin(xyz=(0.270, 0.0, 0.114)), material=metal_mat, name="stem_hinge_base")
    _add_tube(deck, (0.270, 0.050, 0.143), (0.270, 0.090, 0.143), radius=0.018, material=dark_metal_mat, name="stem_hinge_lug_0", extend=0.0)
    _add_tube(deck, (0.270, -0.050, 0.143), (0.270, -0.090, 0.143), radius=0.018, material=dark_metal_mat, name="stem_hinge_lug_1", extend=0.0)
    deck.visual(Box((0.036, 0.075, 0.026)), origin=Origin(xyz=(0.321, 0.0, 0.116)), material=metal_mat, name="fork_crown_mount")

    # Rear fork plates hold the single trailing wheel outside the deck footprint.
    deck.visual(Box((0.085, 0.008, 0.060)), origin=Origin(xyz=(-0.360, 0.033, 0.075)), material=metal_mat, name="rear_dropout_0")
    deck.visual(Box((0.085, 0.008, 0.060)), origin=Origin(xyz=(-0.360, -0.033, 0.075)), material=metal_mat, name="rear_dropout_1")
    _add_tube(deck, (-0.395, 0.022, 0.055), (-0.395, 0.033, 0.055), radius=0.008, material=dark_metal_mat, name="rear_axle_cap_0", extend=0.0)
    _add_tube(deck, (-0.395, -0.022, 0.055), (-0.395, -0.033, 0.055), radius=0.008, material=dark_metal_mat, name="rear_axle_cap_1", extend=0.0)

    handlebar = model.part("handlebar")
    _add_tube(handlebar, (0.0, -0.050, 0.0), (0.0, 0.050, 0.0), radius=0.016, material=dark_metal_mat, name="folding_hinge_barrel", extend=0.0)
    _add_tube(handlebar, (0.012, 0.0, 0.010), (0.055, 0.0, 0.725), radius=0.014, material=metal_mat, name="handlebar_stem", extend=0.005)
    _add_tube(handlebar, (0.055, -0.185, 0.725), (0.055, 0.185, 0.725), radius=0.011, material=metal_mat, name="handlebar_crossbar", extend=0.0)
    _add_tube(handlebar, (0.055, -0.232, 0.725), (0.055, -0.160, 0.725), radius=0.015, material=rubber_mat, name="grip_0", extend=0.0)
    _add_tube(handlebar, (0.055, 0.160, 0.725), (0.055, 0.232, 0.725), radius=0.015, material=rubber_mat, name="grip_1", extend=0.0)
    handlebar.visual(Box((0.050, 0.048, 0.026)), origin=Origin(xyz=(0.018, 0.0, 0.018)), material=dark_metal_mat, name="stem_clamp")

    front_fork = model.part("front_fork")
    _add_tube(front_fork, (-0.026, 0.0, 0.0), (0.026, 0.0, 0.0), radius=0.017, material=dark_metal_mat, name="tilt_crown_barrel", extend=0.0)
    _add_tube(front_fork, (0.000, 0.0, -0.002), (0.018, 0.0, -0.050), radius=0.012, material=metal_mat, name="fork_neck", extend=0.004)
    _add_tube(front_fork, (0.017, 0.0, -0.048), (0.045, 0.092, -0.070), radius=0.010, material=metal_mat, name="fork_arm_0", extend=0.005)
    _add_tube(front_fork, (0.017, 0.0, -0.048), (0.045, -0.092, -0.070), radius=0.010, material=metal_mat, name="fork_arm_1", extend=0.005)
    _add_tube(front_fork, (0.045, 0.087, -0.070), (0.045, 0.106, -0.070), radius=0.008, material=dark_metal_mat, name="front_axle_stub_0", extend=0.0)
    _add_tube(front_fork, (0.045, -0.087, -0.070), (0.045, -0.106, -0.070), radius=0.008, material=dark_metal_mat, name="front_axle_stub_1", extend=0.0)

    front_wheel_0 = model.part("front_wheel_0")
    _add_scooter_wheel(front_wheel_0, prefix="front_wheel_0", tire_mat=urethane_mat, rim_mat=rim_mat, hub_mat=dark_metal_mat)
    front_wheel_1 = model.part("front_wheel_1")
    _add_scooter_wheel(front_wheel_1, prefix="front_wheel_1", tire_mat=urethane_mat, rim_mat=rim_mat, hub_mat=dark_metal_mat)
    rear_wheel = model.part("rear_wheel")
    _add_scooter_wheel(rear_wheel, prefix="rear_wheel", tire_mat=urethane_mat, rim_mat=rim_mat, hub_mat=dark_metal_mat)

    model.articulation(
        "deck_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handlebar,
        origin=Origin(xyz=(0.270, 0.0, 0.143)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.25, friction=0.05),
    )
    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.365, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.35, upper=0.35),
        motion_properties=MotionProperties(damping=0.15, friction=0.03),
    )
    wheel_axis_origin = (0.0, 0.0, math.pi / 2.0)
    model.articulation(
        "front_fork_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_0,
        origin=Origin(xyz=(0.045, 0.126, -0.070), rpy=wheel_axis_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    model.articulation(
        "front_fork_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_1,
        origin=Origin(xyz=(0.045, -0.126, -0.070), rpy=wheel_axis_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.395, 0.0, 0.055), rpy=wheel_axis_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    handlebar = object_model.get_part("handlebar")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_hinge = object_model.get_articulation("deck_to_handlebar")
    fork_tilt = object_model.get_articulation("deck_to_front_fork")

    ctx.check(
        "three rolling wheels are present",
        all(part is not None for part in (front_wheel_0, front_wheel_1, rear_wheel)),
        details="Expected two front wheel links and one rear wheel link.",
    )
    ctx.expect_overlap(front_wheel_0, front_wheel_1, axes="x", min_overlap=0.090, name="front wheels share one transverse axle line")
    ctx.expect_origin_gap(front_wheel_0, front_wheel_1, axis="y", min_gap=0.20, max_gap=0.26, name="front wheels are split left and right")
    ctx.expect_origin_gap(front_wheel_0, rear_wheel, axis="x", min_gap=0.70, max_gap=0.90, name="front wheels sit ahead of rear wheel")

    ctx.expect_contact(deck, front_fork, contact_tol=0.020, name="fork crown is carried by the deck-front mount")
    ctx.expect_contact(deck, handlebar, contact_tol=0.016, name="folding stem hinge is seated between deck lugs")

    rest_bar = ctx.part_element_world_aabb(handlebar, elem="handlebar_crossbar")
    with ctx.pose({stem_hinge: 1.20}):
        folded_bar = ctx.part_element_world_aabb(handlebar, elem="handlebar_crossbar")
    rest_center = None if rest_bar is None else ((rest_bar[0][0] + rest_bar[1][0]) * 0.5, (rest_bar[0][2] + rest_bar[1][2]) * 0.5)
    folded_center = None if folded_bar is None else ((folded_bar[0][0] + folded_bar[1][0]) * 0.5, (folded_bar[0][2] + folded_bar[1][2]) * 0.5)
    ctx.check(
        "handlebar stem folds rearward and downward",
        rest_center is not None
        and folded_center is not None
        and folded_center[0] < rest_center[0] - 0.45
        and folded_center[1] < rest_center[1] - 0.35,
        details=f"rest_center_xz={rest_center}, folded_center_xz={folded_center}",
    )

    at_rest_0 = ctx.part_world_position(front_wheel_0)
    at_rest_1 = ctx.part_world_position(front_wheel_1)
    with ctx.pose({fork_tilt: 0.30}):
        tilted_0 = ctx.part_world_position(front_wheel_0)
        tilted_1 = ctx.part_world_position(front_wheel_1)
    ctx.check(
        "fork tilt leans the paired front wheels",
        at_rest_0 is not None
        and at_rest_1 is not None
        and tilted_0 is not None
        and tilted_1 is not None
        and abs((tilted_0[2] - tilted_1[2]) - (at_rest_0[2] - at_rest_1[2])) > 0.055,
        details=f"rest=({at_rest_0}, {at_rest_1}), tilted=({tilted_0}, {tilted_1})",
    )

    return ctx.report()


object_model = build_object_model()
