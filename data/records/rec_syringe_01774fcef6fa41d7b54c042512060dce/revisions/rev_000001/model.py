from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _x_cylinder(
    radius: float,
    length: float,
    center_x: float,
    *,
    radial_segments: int = 32,
) -> MeshGeometry:
    return (
        CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
        .rotate_y(math.pi / 2.0)
        .translate(center_x, 0.0, 0.0)
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _build_barrel_shell() -> MeshGeometry:
    outer_profile = [
        (0.0094, 0.0000),
        (0.0094, 0.1020),
        (0.0072, 0.1085),
        (0.0037, 0.1180),
        (0.0017, 0.1270),
    ]
    inner_profile = [
        (0.0077, 0.0012),
        (0.0077, 0.0995),
        (0.0022, 0.1160),
        (0.0008, 0.1245),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)


def _build_plunger_seal() -> MeshGeometry:
    core = _x_cylinder(radius=0.0063, length=0.0120, center_x=0.0110, radial_segments=28)
    rear_lip = _x_cylinder(radius=0.00715, length=0.0025, center_x=0.0070, radial_segments=28)
    front_lip = _x_cylinder(radius=0.00715, length=0.0025, center_x=0.0150, radial_segments=28)
    return _merge_meshes(core, rear_lip, front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_barrel = model.material("clear_barrel", rgba=(0.87, 0.92, 0.98, 0.40))
    white_plastic = model.material("white_plastic", rgba=(0.95, 0.96, 0.97, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.83, 0.85, 0.88, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.14, 0.15, 0.16, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_shell(), "barrel_shell"),
        material=clear_barrel,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.006, 0.024, 0.006)),
        origin=Origin(xyz=(0.003, 0.018, 0.0)),
        material=clear_barrel,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.006, 0.024, 0.006)),
        origin=Origin(xyz=(0.003, -0.018, 0.0)),
        material=clear_barrel,
        name="finger_flange_right",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.130, 0.060, 0.024)),
        mass=0.035,
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0022, length=0.090),
        origin=Origin(xyz=(-0.037, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0050, length=0.010),
        origin=Origin(xyz=(-0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray_plastic,
        name="thumb_hub",
    )
    plunger.visual(
        Cylinder(radius=0.013, length=0.0035),
        origin=Origin(xyz=(-0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        mesh_from_geometry(_build_plunger_seal(), "plunger_seal"),
        material=black_rubber,
        name="plunger_seal",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.105, 0.028, 0.028)),
        mass=0.010,
        origin=Origin(xyz=(-0.037, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.082,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_seal",
        outer_elem="barrel_shell",
        margin=0.0004,
        name="seal stays centered in barrel at rest",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_seal",
        elem_b="barrel_shell",
        min_overlap=0.008,
        name="seal remains inserted at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.expect_within(
                plunger,
                barrel,
                axes="yz",
                inner_elem="plunger_seal",
                outer_elem="barrel_shell",
                margin=0.0004,
                name="seal stays centered when depressed",
            )
            ctx.expect_overlap(
                plunger,
                barrel,
                axes="x",
                elem_a="plunger_seal",
                elem_b="barrel_shell",
                min_overlap=0.008,
                name="seal remains inserted when depressed",
            )
            ctx.expect_gap(
                barrel,
                plunger,
                axis="x",
                positive_elem="barrel_shell",
                negative_elem="thumb_pad",
                min_gap=0.0015,
                max_gap=0.020,
                name="thumb pad stays behind barrel rear when fully depressed",
            )
            extended_pos = ctx.part_world_position(plunger)

        ctx.check(
            "plunger translates deeper into the barrel",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.05,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
