from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=96,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_turbofan_engine")

    nacelle_blue = model.material("nacelle_blue", rgba=(0.10, 0.15, 0.24, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    fan_titanium = model.material("fan_titanium", rgba=(0.50, 0.53, 0.55, 1.0))
    ceramic = model.material("hot_section_ceramic", rgba=(0.34, 0.32, 0.29, 1.0))

    nacelle = model.part("nacelle")

    # The engine centerline is the model X axis.  Lathed meshes are authored
    # about local Z and rotated so their axes land on world X.
    axis_to_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    nacelle.visual(
        _shell_mesh(
            outer_profile=[
                (0.94, -1.62),
                (0.97, -1.54),
                (0.90, -1.42),
                (0.90, -1.20),
                (0.90, 0.85),
                (0.84, 1.30),
                (0.71, 1.40),
                (0.72, 1.50),
                (0.76, 1.58),
            ],
            inner_profile=[
                (0.67, -1.62),
                (0.66, -1.54),
                (0.70, -1.42),
                (0.70, -1.15),
                (0.67, 0.90),
                (0.58, 1.30),
                (0.55, 1.40),
                (0.54, 1.50),
                (0.48, 1.58),
            ],
            name="nacelle_shell",
        ),
        origin=axis_to_x,
        material=nacelle_blue,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.00, -1.14),
                    (0.18, -1.06),
                    (0.31, -0.45),
                    (0.34, 0.65),
                    (0.26, 1.28),
                    (0.11, 1.50),
                    (0.00, 1.56),
                ],
                segments=72,
            ),
            "core_body",
        ),
        origin=axis_to_x,
        material=dark_metal,
        name="core_body",
    )
    nacelle.visual(
        Cylinder(radius=0.095, length=0.66),
        origin=Origin(xyz=(-1.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="fan_shaft",
    )

    for station_x, core_r, duct_r, prefix in (
        (-0.92, 0.17, 0.74, "front_stator"),
        (0.72, 0.25, 0.66, "rear_stator"),
    ):
        for i in range(6):
            theta = 2.0 * math.pi * i / 6.0
            c = math.cos(theta)
            s = math.sin(theta)
            _add_member(
                nacelle,
                (station_x, core_r * c, core_r * s),
                (station_x, duct_r * c, duct_r * s),
                0.022 if prefix == "front_stator" else 0.018,
                satin_metal,
                name=f"{prefix}_{i}",
            )

    fan = model.part("fan")
    fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.635,
                hub_radius=0.185,
                blade_count=18,
                thickness=0.16,
                blade_pitch_deg=33.0,
                blade_sweep_deg=31.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.16),
                hub=FanRotorHub(style="spinner", bore_diameter=0.16),
            ),
            "fan_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fan_titanium,
        name="fan_rotor",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-1.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=120.0),
    )

    petal_count = 12
    hinge_radius = 0.69
    for i in range(petal_count):
        theta = 2.0 * math.pi * i / petal_count
        petal = model.part(f"nozzle_petal_{i}")
        petal.visual(
            Cylinder(radius=0.055, length=0.18),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name="hinge_barrel",
        )
        petal.visual(
            Box((0.16, 0.16, 0.060)),
            origin=Origin(xyz=(0.075, 0.0, -0.020), rpy=(0.0, 0.08, 0.0)),
            material=satin_metal,
            name="hinge_neck",
        )
        petal.visual(
            Box((0.66, 0.20, 0.035)),
            origin=Origin(xyz=(0.42, 0.0, -0.105), rpy=(0.0, 0.17, 0.0)),
            material=ceramic,
            name="petal_panel",
        )
        petal.visual(
            Box((0.52, 0.035, 0.045)),
            origin=Origin(xyz=(0.45, 0.0, -0.072), rpy=(0.0, 0.17, 0.0)),
            material=satin_metal,
            name="stiffener_rib",
        )

        joint_name = f"petal_pivot_{i}"
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=nacelle,
            child=petal,
            origin=Origin(
                xyz=(1.50, hinge_radius * math.cos(theta), hinge_radius * math.sin(theta)),
                rpy=(theta - math.pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.22, upper=0.34),
            mimic=None if i == 0 else Mimic("petal_pivot_0"),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    fan_spin = object_model.get_articulation("fan_spin")
    petal_0 = object_model.get_part("nozzle_petal_0")
    petal_pivot_0 = object_model.get_articulation("petal_pivot_0")

    ctx.expect_origin_distance(
        fan,
        nacelle,
        axes="yz",
        max_dist=0.002,
        name="fan hub lies on nacelle centerline",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_rotor",
        outer_elem="nacelle_shell",
        margin=0.01,
        name="fan disk fits inside cylindrical nacelle",
    )
    ctx.allow_overlap(
        nacelle,
        fan,
        elem_a="fan_shaft",
        elem_b="fan_rotor",
        reason="The rotating fan hub is intentionally captured on the fixed center shaft proxy.",
    )
    ctx.expect_overlap(
        nacelle,
        fan,
        axes="xyz",
        elem_a="fan_shaft",
        elem_b="fan_rotor",
        min_overlap=0.05,
        name="fan hub is retained on center shaft",
    )

    fan_pos = ctx.part_world_position(fan)
    with ctx.pose({fan_spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(fan)
    ctx.check(
        "fan spin keeps rotor on centerline",
        fan_pos is not None and spun_pos is not None and fan_pos == spun_pos,
        details=f"rest={fan_pos}, spun={spun_pos}",
    )

    for i in range(12):
        petal = object_model.get_part(f"nozzle_petal_{i}")
        joint = object_model.get_articulation(f"petal_pivot_{i}")
        pos = ctx.part_world_position(petal)
        radial = math.hypot(pos[1], pos[2]) if pos is not None else None
        ctx.check(
            f"nozzle petal {i} mounted on rear ring",
            pos is not None and abs(pos[0] - 1.50) < 0.002 and radial is not None and 0.67 < radial < 0.71,
            details=f"pos={pos}, radial={radial}",
        )
        ctx.allow_overlap(
            nacelle,
            petal,
            elem_a="nacelle_shell",
            elem_b="hinge_barrel",
            reason="Each nozzle petal hinge barrel is captured inside the rear band portion of the stationary nacelle shell.",
        )
        ctx.allow_overlap(
            nacelle,
            petal,
            elem_a="nacelle_shell",
            elem_b="hinge_neck",
            reason="Each short hinge neck is locally seated into the rear exhaust ring to support the petal pivot.",
        )
        ctx.expect_overlap(
            petal,
            nacelle,
            axes="xyz",
            elem_a="hinge_barrel",
            elem_b="nacelle_shell",
            min_overlap=0.02,
            name=f"petal {i} hinge is retained in rear band",
        )
        ctx.expect_overlap(
            petal,
            nacelle,
            axes="xyz",
            elem_a="hinge_neck",
            elem_b="nacelle_shell",
            min_overlap=0.02,
            name=f"petal {i} hinge neck seats in rear ring",
        )
        ctx.check(
            f"petal {i} pivot is a short tangential hinge",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    rest_box = ctx.part_element_world_aabb(petal_0, elem="petal_panel")
    with ctx.pose({petal_pivot_0: 0.30}):
        closed_box = ctx.part_element_world_aabb(petal_0, elem="petal_panel")
    ctx.check(
        "positive nozzle command closes petals inward",
        rest_box is not None
        and closed_box is not None
        and closed_box[0][1] < rest_box[0][1] - 0.15,
        details=f"rest={rest_box}, commanded={closed_box}",
    )

    return ctx.report()


object_model = build_object_model()
