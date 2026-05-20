from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
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
    superellipse_profile,
)


DECK_LENGTH = 0.95
DECK_WIDTH = 0.30
DECK_THICKNESS = 0.036
DECK_Z = 0.180
DECK_UNDERSIDE_Z = DECK_Z - DECK_THICKNESS / 2.0
DECK_TOP_Z = DECK_Z + DECK_THICKNESS / 2.0

TRUCK_X = 0.320
BASE_PLATE_THICKNESS = 0.014
STEER_Z = -0.056
WHEEL_CENTER_Y = 0.194
WHEEL_CENTER_Z = -0.046
WHEEL_RADIUS = 0.058
WHEEL_WIDTH = 0.052


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    for existing in model.materials:
        if existing.name == name:
            return existing
    return model.material(name, rgba=rgba)


def _cylinder_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Return a cylinder and the transform that aligns its local Z axis to Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_deck_visuals(model: ArticulatedObject, deck) -> None:
    deck_mat = _mat(model, "oil_black_grip", (0.015, 0.018, 0.018, 1.0))
    tread_mat = _mat(model, "coarse_safety_tread", (0.035, 0.040, 0.040, 1.0))
    steel_mat = _mat(model, "brushed_steel", (0.58, 0.60, 0.58, 1.0))
    yellow_mat = _mat(model, "safety_yellow", (1.0, 0.74, 0.06, 1.0))

    deck_profile = superellipse_profile(DECK_LENGTH, DECK_WIDTH, exponent=3.2, segments=96)
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(deck_profile, DECK_THICKNESS, center=True),
            "armored_rounded_deck",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material=deck_mat,
        name="deck_shell",
    )

    # Raised armor rails are intentionally utilitarian: they stiffen the long
    # edges and give a boot-visible boundary without becoming decorative.
    for idx, y in enumerate((-DECK_WIDTH / 2.0 + 0.010, DECK_WIDTH / 2.0 - 0.010)):
        deck.visual(
            Box((0.820, 0.022, 0.022)),
            origin=Origin(xyz=(0.0, y, DECK_Z + 0.006)),
            material=steel_mat,
            name=f"edge_rail_{idx}",
        )

    # Three replaceable anti-slip tread plates sit slightly proud and bite into
    # the deck shell, making them supported instead of floating decals.
    for idx, x in enumerate((-0.245, 0.0, 0.245)):
        deck.visual(
            Box((0.205, 0.220, 0.005)),
            origin=Origin(xyz=(x, 0.0, DECK_TOP_Z + 0.0015)),
            material=tread_mat,
            name=f"tread_plate_{idx}",
        )

    # Underside doubler plates and diagonal braces make the high-cycle load path
    # from deck to trucks visible.
    for idx, x in enumerate((-TRUCK_X, TRUCK_X)):
        deck.visual(
            Box((0.250, 0.230, 0.007)),
            origin=Origin(xyz=(x, 0.0, DECK_TOP_Z + 0.0035)),
            material=steel_mat,
            name=f"truck_doubler_{idx}",
        )
        for brace_idx, yaw in enumerate((0.54, -0.54)):
            deck.visual(
                Box((0.285, 0.014, 0.009)),
                origin=Origin(xyz=(x, 0.0, DECK_TOP_Z + 0.0075), rpy=(0.0, 0.0, yaw)),
                material=steel_mat,
                name=f"cross_brace_{idx}_{brace_idx}",
            )
        for bolt_idx, (bx, by) in enumerate(((-0.065, -0.050), (-0.065, 0.050), (0.065, -0.050), (0.065, 0.050))):
            deck.visual(
                Cylinder(radius=0.010, length=0.005),
                origin=Origin(xyz=(x + bx, by, DECK_TOP_Z + 0.0105)),
                material=yellow_mat,
                name=f"top_bolt_{idx}_{bolt_idx}",
            )


def _add_base_visuals(model: ArticulatedObject, base, prefix: str) -> None:
    steel = _mat(model, "base_steel", (0.42, 0.44, 0.43, 1.0))
    black = _mat(model, "black_oxide", (0.02, 0.022, 0.022, 1.0))
    bushing = _mat(model, "amber_bushing", (0.95, 0.48, 0.10, 1.0))
    yellow = _mat(model, "lockout_yellow", (1.0, 0.72, 0.03, 1.0))

    base.visual(
        Box((0.205, 0.160, BASE_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -BASE_PLATE_THICKNESS / 2.0)),
        material=steel,
        name="base_plate",
    )
    base.visual(
        Box((0.150, 0.095, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="kingpin_boss",
    )

    # Visible fastener logic: underside nuts line up with the deck-top bolts.
    for bolt_idx, (bx, by) in enumerate(((-0.065, -0.050), (-0.065, 0.050), (0.065, -0.050), (0.065, 0.050))):
        base.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(bx, by, -BASE_PLATE_THICKNESS - 0.003)),
            material=black,
            name=f"bottom_nut_{bolt_idx}",
        )

    # Kingpin and bushing stack are coaxial with the truck steering joint.
    base.visual(
        Cylinder(radius=0.0065, length=0.051),
        origin=Origin(xyz=(0.0, 0.0, -0.0295)),
        material=black,
        name="kingpin",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0185)),
        material=steel,
        name="top_washer",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=bushing,
        name="top_bushing",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=bushing,
        name="lower_bushing",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=steel,
        name="lower_washer",
    )

    # Solid stop blocks are placed on the fixed base to make the steering limit
    # physical rather than only a numeric joint limit.
    for stop_idx, x in enumerate((-0.058, 0.058)):
        base.visual(
            Box((0.020, 0.026, 0.032)),
            origin=Origin(xyz=(x, 0.067, -0.034)),
            material=yellow,
            name=f"travel_stop_{stop_idx}",
        )

    # A guided sliding lockout pin lives in a supported sleeve on one side of
    # each truck. The stanchion is offset so only the sleeve intentionally
    # captures the moving pin.
    base.visual(
        Box((0.044, 0.034, 0.026)),
        origin=Origin(xyz=(0.065, -0.100, -0.045)),
        material=yellow,
        name="lockout_guide",
    )
    base.visual(
        Box((0.020, 0.036, 0.038)),
        origin=Origin(xyz=(0.091, -0.100, -0.028)),
        material=yellow,
        name="lockout_stanchion",
    )
    base.visual(
        Box((0.064, 0.056, 0.012)),
        origin=Origin(xyz=(0.075, -0.083, -0.018)),
        material=yellow,
        name="lockout_outrigger",
    )


def _add_hanger_visuals(model: ArticulatedObject, hanger) -> None:
    steel = _mat(model, "hanger_cast_steel", (0.34, 0.36, 0.36, 1.0))
    yellow = _mat(model, "guard_yellow", (1.0, 0.72, 0.03, 1.0))
    black = _mat(model, "black_oxide_hardware", (0.015, 0.015, 0.014, 1.0))

    hanger.visual(
        Box((0.132, 0.078, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=steel,
        name="hanger_body",
    )
    hanger.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=steel,
        name="pivot_cup",
    )

    axle_geom, axle_origin = _cylinder_y(0.0075, 0.420)
    hanger.visual(
        axle_geom,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z), rpy=axle_origin.rpy),
        material=black,
        name="axle",
    )
    for idx, y in enumerate((-0.150, 0.150)):
        collar_geom, collar_origin = _cylinder_y(0.014, 0.014)
        hanger.visual(
            collar_geom,
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z), rpy=collar_origin.rpy),
            material=black,
            name=f"axle_collar_{idx}",
        )

    # Moving stop ears on the hanger approach the fixed yellow base stops at the
    # travel extremes.
    for idx, x in enumerate((-0.040, 0.040)):
        hanger.visual(
            Box((0.020, 0.026, 0.022)),
            origin=Origin(xyz=(x, 0.040, -0.002)),
            material=yellow,
            name=f"stop_ear_{idx}",
        )

    # Keeper block receives the sliding lockout pin at the engaged end of its
    # travel; it is part of the steering hanger so the pin visibly locks steer.
    hanger.visual(
        Box((0.030, 0.016, 0.026)),
        origin=Origin(xyz=(0.065, -0.010, 0.006)),
        material=yellow,
        name="lockout_keeper",
    )

    # Wheel guards are carried by the hanger with bridge struts, so they steer
    # with the axle and never read as floating fenders.
    for idx, sign in enumerate((-1.0, 1.0)):
        y = sign * WHEEL_CENTER_Y
        hanger.visual(
            Box((0.118, 0.074, 0.007)),
            origin=Origin(xyz=(0.0, y, 0.026)),
            material=yellow,
            name=f"guard_crown_{idx}",
        )
        hanger.visual(
            Box((0.112, 0.007, 0.074)),
            origin=Origin(xyz=(0.0, sign * 0.223, -0.014)),
            material=yellow,
            name=f"guard_side_{idx}",
        )
        hanger.visual(
            Box((0.034, 0.155, 0.010)),
            origin=Origin(xyz=(0.0, sign * 0.110, 0.022)),
            material=yellow,
            name=f"guard_strut_{idx}",
        )
        hanger.visual(
            Box((0.034, 0.040, 0.036)),
            origin=Origin(xyz=(0.0, sign * 0.052, 0.004)),
            material=yellow,
            name=f"guard_pillar_{idx}",
        )


def _wheel_meshes(name: str):
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.040,
            WHEEL_WIDTH * 0.76,
            rim=WheelRim(inner_radius=0.027, flange_height=0.004, flange_thickness=0.003, bead_seat_depth=0.002),
            hub=WheelHub(
                radius=0.018,
                width=0.028,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.024, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        f"{name}_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.041,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.004, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.018),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        f"{name}_tire",
    )
    return rim_mesh, tire_mesh


def _add_wheel_visuals(model: ArticulatedObject, wheel, name: str) -> None:
    tire_mat = _mat(model, "industrial_rubber", (0.018, 0.018, 0.016, 1.0))
    rim_mat = _mat(model, "zinc_rim", (0.72, 0.72, 0.68, 1.0))
    rim_mesh, tire_mesh = _wheel_meshes(name)
    wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_mat,
        name="tire",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_mat,
        name="rim",
    )


def _add_lockout_visuals(model: ArticulatedObject, lockout) -> None:
    black = _mat(model, "lockout_black", (0.02, 0.021, 0.020, 1.0))
    red = _mat(model, "lockout_red", (0.75, 0.06, 0.035, 1.0))
    pin_geom, pin_origin = _cylinder_y(0.0055, 0.070)
    lockout.visual(
        pin_geom,
        origin=pin_origin,
        material=black,
        name="lockout_pin",
    )
    lockout.visual(
        Box((0.028, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=red,
        name="pull_tab",
    )


def _build_truck(model: ArticulatedObject, deck, prefix: str, x: float, wheel_start_idx: int) -> None:
    base = model.part(f"{prefix}_base")
    _add_base_visuals(model, base, prefix)
    model.articulation(
        f"{prefix}_base_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=base,
        origin=Origin(xyz=(x, 0.0, DECK_UNDERSIDE_Z)),
    )

    hanger = model.part(f"{prefix}_hanger")
    _add_hanger_visuals(model, hanger)
    model.articulation(
        f"{prefix}_steer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hanger,
        origin=Origin(xyz=(0.0, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-0.30, upper=0.30),
        motion_properties=MotionProperties(damping=1.2, friction=0.35),
    )

    lockout = model.part(f"{prefix}_lockout")
    _add_lockout_visuals(model, lockout)
    model.articulation(
        f"{prefix}_lockout_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lockout,
        origin=Origin(xyz=(0.065, -0.100, -0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.045),
        motion_properties=MotionProperties(damping=4.0, friction=1.0),
    )

    for side_idx, y in enumerate((-WHEEL_CENTER_Y, WHEEL_CENTER_Y)):
        wheel_idx = wheel_start_idx + side_idx
        wheel = model.part(f"wheel_{wheel_idx}")
        _add_wheel_visuals(model, wheel, f"wheel_{wheel_idx}")
        model.articulation(
            f"wheel_{wheel_idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel,
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=30.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_skateboard")
    deck = model.part("deck")
    _add_deck_visuals(model, deck)

    _build_truck(model, deck, "front", TRUCK_X, 0)
    _build_truck(model, deck, "rear", -TRUCK_X, 2)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_base = object_model.get_part("front_base")
    rear_base = object_model.get_part("rear_base")
    front_lockout = object_model.get_part("front_lockout")
    rear_lockout = object_model.get_part("rear_lockout")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")

    # The only intentional current-pose interpenetration is the captured sliding
    # lockout pin represented by a solid sleeve. It is scoped to the guide/pin
    # elements and checked as a retained sliding fit.
    for prefix, base, lockout, hanger in (
        ("front", front_base, front_lockout, front_hanger),
        ("rear", rear_base, rear_lockout, rear_hanger),
    ):
        ctx.allow_overlap(
            base,
            lockout,
            elem_a="lockout_guide",
            elem_b="lockout_pin",
            reason="The lockout pin intentionally rides inside a solid guide sleeve proxy.",
        )
        ctx.expect_within(
            lockout,
            base,
            axes="xz",
            inner_elem="lockout_pin",
            outer_elem="lockout_guide",
            margin=0.002,
            name=f"{prefix} lockout pin centered in guide",
        )
        ctx.expect_overlap(
            lockout,
            base,
            axes="y",
            elem_a="lockout_pin",
            elem_b="lockout_guide",
            min_overlap=0.020,
            name=f"{prefix} lockout pin retained in guide",
        )

        slide = object_model.get_articulation(f"{prefix}_lockout_slide")
        rest_y = ctx.part_world_position(lockout)[1]
        with ctx.pose({slide: 0.045}):
            engaged_y = ctx.part_world_position(lockout)[1]
            ctx.expect_gap(
                hanger,
                lockout,
                axis="y",
                positive_elem="lockout_keeper",
                negative_elem="lockout_pin",
                min_gap=0.0,
                max_gap=0.006,
                name=f"{prefix} lockout reaches keeper",
            )
        ctx.check(
            f"{prefix} lockout slides toward keeper",
            engaged_y > rest_y + 0.035,
            details=f"rest_y={rest_y}, engaged_y={engaged_y}",
        )

    for prefix in ("front", "rear"):
        steer = object_model.get_articulation(f"{prefix}_steer")
        ctx.check(
            f"{prefix} truck has safety steer limits",
            steer.motion_limits is not None
            and steer.motion_limits.lower == -0.30
            and steer.motion_limits.upper == 0.30,
            details=str(steer.motion_limits),
        )
        hanger = object_model.get_part(f"{prefix}_hanger")
        base = object_model.get_part(f"{prefix}_base")
        ctx.expect_contact(
            hanger,
            base,
            elem_a="pivot_cup",
            elem_b="lower_washer",
            contact_tol=0.0015,
            name=f"{prefix} bushing stack seats hanger cup",
        )

    for idx in range(4):
        wheel = object_model.get_part(f"wheel_{idx}")
        parent_hanger = front_hanger if idx < 2 else rear_hanger
        ctx.expect_within(
            wheel,
            parent_hanger,
            axes="xz",
            inner_elem="rim",
            outer_elem="axle",
            margin=0.050,
            name=f"wheel_{idx} bore follows axle line",
        )
        spin = object_model.get_articulation(f"wheel_{idx}_spin")
        ctx.check(
            f"wheel_{idx} spins about axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    return ctx.report()


object_model = build_object_model()
