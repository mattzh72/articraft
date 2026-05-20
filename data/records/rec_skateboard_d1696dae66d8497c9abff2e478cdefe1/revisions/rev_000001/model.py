from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    ExtrudeGeometry,
)


def _skate_deck_geometry() -> MeshGeometry:
    """A single connected, slightly kicked skateboard deck mesh."""
    length = 0.82
    width = 0.215
    thickness = 0.014
    nose_len = 0.135
    base_z = 0.116

    geom = MeshGeometry()
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    nx = 31
    ny = 13

    for i in range(nx):
        u = i / (nx - 1)
        x = (u - 0.5) * length
        end = max(0.0, abs(x) - (length / 2.0 - nose_len)) / nose_len
        # Rounded nose/tail in plan view.
        section_width = width * (0.43 + 0.57 * math.cos(end * math.pi / 2.0))
        # Subtle upward kick at both ends.
        kick = 0.042 * (math.sin(end * math.pi / 2.0) ** 1.7)
        # A shallow longitudinal rocker keeps the middle slightly low.
        rocker = 0.004 * math.cos(u * 2.0 * math.pi)
        zc = base_z + kick - rocker

        top_row = []
        bottom_row = []
        for j in range(ny):
            eta = -1.0 + 2.0 * j / (ny - 1)
            # The parameterized grid is rectangular in (x, eta), but narrows
            # toward the nose and tail to make a real board outline.
            y = (section_width / 2.0) * math.copysign(abs(eta) ** 0.62, eta)
            transverse_concave = 0.0045 * (abs(eta) ** 1.8)
            edge_rounding = 0.0015 * (abs(eta) ** 8.0)
            top_row.append(geom.add_vertex(x, y, zc + thickness / 2.0 + transverse_concave - edge_rounding))
            bottom_row.append(geom.add_vertex(x, y, zc - thickness / 2.0 + 0.0015 * transverse_concave + edge_rounding))
        top.append(top_row)
        bottom.append(bottom_row)

    for i in range(nx - 1):
        for j in range(ny - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    # Long rounded rails.
    for i in range(nx - 1):
        for j in (0, ny - 1):
            geom.add_face(bottom[i][j], bottom[i + 1][j], top[i + 1][j])
            geom.add_face(bottom[i][j], top[i + 1][j], top[i][j])

    # Nose and tail caps.
    for i in (0, nx - 1):
        for j in range(ny - 1):
            geom.add_face(bottom[i][j], top[i][j], top[i][j + 1])
            geom.add_face(bottom[i][j], top[i][j + 1], bottom[i][j + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_truck_skateboard")

    maple = Material("sealed_maple", rgba=(0.72, 0.48, 0.27, 1.0))
    grip = Material("black_grip_tape", rgba=(0.015, 0.014, 0.013, 1.0))
    black = Material("black_hardware", rgba=(0.02, 0.02, 0.018, 1.0))
    aluminum = Material("cast_aluminum", rgba=(0.68, 0.69, 0.66, 1.0))
    steel = Material("polished_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    urethane = Material("cream_urethane", rgba=(0.92, 0.88, 0.72, 1.0))
    hub_white = Material("white_bearing_hub", rgba=(0.95, 0.95, 0.90, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_skate_deck_geometry(), "kicked_maple_deck"),
        material=maple,
        name="maple_deck",
    )
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.68, 0.177, 0.070, corner_segments=10),
                0.002,
            ),
            "black_grip_tape",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=grip,
        name="grip_tape",
    )

    # Fixed truck supports, pads, kingpin bosses, mounting bolt heads, and the
    # required brace tying the two main supports together near the base.
    for truck_x, prefix in ((0.255, "front"), (-0.255, "rear")):
        deck.visual(
            Box((0.130, 0.088, 0.009)),
            origin=Origin(xyz=(truck_x, 0.0, 0.102)),
            material=aluminum,
            name=f"{prefix}_baseplate",
        )
        deck.visual(
            Box((0.122, 0.080, 0.012)),
            origin=Origin(xyz=(truck_x, 0.0, 0.111)),
            material=black,
            name=f"{prefix}_riser_pad",
        )
        deck.visual(
            Cylinder(radius=0.015, length=0.035),
            origin=Origin(xyz=(truck_x, 0.0, 0.084), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"{prefix}_kingpin",
        )
        for bx in (-0.038, 0.038):
            for by in (-0.026, 0.026):
                deck.visual(
                    Cylinder(radius=0.0055, length=0.030),
                    origin=Origin(xyz=(truck_x + bx, by, 0.111)),
                    material=black,
                    name=f"{prefix}_bolt_{bx:+.0e}_{by:+.0e}",
                )

    deck.visual(
        Box((0.405, 0.028, 0.011)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=steel,
        name="fixed_brace",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.034,
            0.040,
            inner_radius=0.0215,
            carcass=TireCarcass(belt_width_ratio=0.82, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.0015, count=18, land_ratio=0.72),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "smooth_skate_wheel_urethane",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.022,
            0.036,
            rim=WheelRim(inner_radius=0.014, flange_height=0.002, flange_thickness=0.002),
            hub=WheelHub(radius=0.012, width=0.030, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "skate_wheel_hub",
    )

    for truck_x, prefix in ((0.255, "front"), (-0.255, "rear")):
        hanger = model.part(f"{prefix}_hanger")
        # The hanger frame is on the steering kingpin.  The axle sits below it.
        hanger.visual(
            Box((0.075, 0.072, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=aluminum,
            name="hanger_body",
        )
        hanger.visual(
            Cylinder(radius=0.014, length=0.142),
            origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="crossbar",
        )
        hanger.visual(
            Cylinder(radius=0.005, length=0.310),
            origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle",
        )
        hanger.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=black,
            name="bushing",
        )

        model.articulation(
            f"{prefix}_hanger_steer",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=hanger,
            origin=Origin(xyz=(truck_x, 0.0, 0.080)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.42, upper=0.42),
        )

        for index, wheel_y in enumerate((-0.155, 0.155)):
            wheel = model.part(f"{prefix}_wheel_{index}")
            wheel.visual(
                tire_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=urethane,
                name="urethane_tire",
            )
            wheel.visual(
                rim_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=hub_white,
                name="bearing_hub",
            )
            model.articulation(
                f"{prefix}_wheel_{index}_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, wheel_y, -0.040)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=3.0, velocity=50.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")

    for prefix in ("front", "rear"):
        ctx.allow_overlap(
            "deck",
            f"{prefix}_hanger",
            elem_a=f"{prefix}_kingpin",
            elem_b="bushing",
            reason="The rubber bushing is intentionally captured around the kingpin on a standard truck.",
        )
        for index in (0, 1):
            ctx.allow_overlap(
                f"{prefix}_hanger",
                f"{prefix}_wheel_{index}",
                elem_a="axle",
                elem_b="bearing_hub",
                reason="The steel axle intentionally passes through the wheel bearing hub.",
            )

    front_steer = object_model.get_articulation("front_hanger_steer")
    rear_steer = object_model.get_articulation("rear_hanger_steer")
    ctx.check(
        "truck hangers steer on revolute kingpins",
        front_steer.articulation_type == ArticulationType.REVOLUTE
        and rear_steer.articulation_type == ArticulationType.REVOLUTE
        and front_steer.motion_limits is not None
        and rear_steer.motion_limits is not None
        and front_steer.motion_limits.lower < 0.0 < front_steer.motion_limits.upper,
        details=f"front={front_steer}, rear={rear_steer}",
    )

    spin_joints = [
        object_model.get_articulation("front_wheel_0_spin"),
        object_model.get_articulation("front_wheel_1_spin"),
        object_model.get_articulation("rear_wheel_0_spin"),
        object_model.get_articulation("rear_wheel_1_spin"),
    ]
    ctx.check(
        "four wheel joints are continuous spins",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in spin_joints),
        details=", ".join(j.name for j in spin_joints),
    )

    for prefix in ("front", "rear"):
        hanger = object_model.get_part(f"{prefix}_hanger")
        ctx.expect_gap(
            deck,
            hanger,
            axis="z",
            min_gap=0.020,
            positive_elem="maple_deck",
            negative_elem="hanger_body",
            name=f"{prefix} hanger is below the deck",
        )
        ctx.expect_overlap(
            deck,
            hanger,
            axes="z",
            min_overlap=0.010,
            elem_a=f"{prefix}_kingpin",
            elem_b="bushing",
            name=f"{prefix} kingpin captures the truck bushing",
        )
        for index in (0, 1):
            wheel = object_model.get_part(f"{prefix}_wheel_{index}")
            ctx.expect_overlap(
                wheel,
                hanger,
                axes="z",
                min_overlap=0.009,
                elem_a="bearing_hub",
                elem_b="axle",
                name=f"{prefix} wheel {index} is centered on its axle height",
            )
            ctx.expect_overlap(
                wheel,
                hanger,
                axes="y",
                min_overlap=0.006,
                elem_a="bearing_hub",
                elem_b="axle",
                name=f"{prefix} wheel {index} is retained on the axle",
            )
            spin = object_model.get_articulation(f"{prefix}_wheel_{index}_spin")
            ctx.check(
                f"{prefix} wheel {index} spin axis follows axle",
                tuple(round(v, 3) for v in spin.axis) == (0.0, 1.0, 0.0),
                details=f"axis={spin.axis}",
            )

    # A decisive pose check: steering swings a truck/wheel pair sideways while
    # leaving the wheel spin joints independent.
    front_wheel = object_model.get_part("front_wheel_0")
    rest_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({front_steer: 0.35}):
        turned_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front truck steering moves the wheel about the kingpin",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[0] - rest_pos[0]) > 0.010,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
