from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_OUTER_RADIUS = 0.0086
BARREL_INNER_RADIUS = 0.0071
BARREL_REAR_Z = 0.094
SLIDE_TRAVEL = 0.054


def _build_barrel_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0012, 0.000),
            (0.0018, 0.004),
            (0.0031, 0.009),
            (0.0045, 0.012),
            (BARREL_OUTER_RADIUS, 0.017),
            (BARREL_OUTER_RADIUS, BARREL_REAR_Z),
        ],
        [
            (0.00045, 0.000),
            (0.00075, 0.004),
            (0.0013, 0.009),
            (0.0019, 0.012),
            (BARREL_INNER_RADIUS, 0.017),
            (BARREL_INNER_RADIUS, BARREL_REAR_Z),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_guide_module():
    collar = LatheGeometry.from_shell_profiles(
        [
            (0.0108, -0.006),
            (0.0112, 0.000),
            (0.0112, 0.010),
        ],
        [
            (0.0089, -0.006),
            (0.0089, 0.010),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    web = LatheGeometry.from_shell_profiles(
        [
            (0.0108, 0.000),
            (0.0108, 0.002),
        ],
        [
            (0.0046, 0.000),
            (0.0046, 0.002),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    guide_sleeve = LatheGeometry.from_shell_profiles(
        [
            (0.0046, 0.002),
            (0.0046, 0.015),
        ],
        [
            (0.0022, 0.002),
            (0.0022, 0.015),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )

    left_bridge = BoxGeometry((0.006, 0.006, 0.008)).translate(-0.0138, 0.0, 0.002)
    right_bridge = BoxGeometry((0.006, 0.006, 0.008)).translate(0.0138, 0.0, 0.002)
    left_pad = BoxGeometry((0.015, 0.011, 0.003)).translate(-0.0240, 0.0, 0.004)
    right_pad = BoxGeometry((0.015, 0.011, 0.003)).translate(0.0240, 0.0, 0.004)

    collar.merge(left_bridge)
    collar.merge(right_bridge)
    collar.merge(left_pad)
    collar.merge(right_pad)
    collar.merge(web)
    collar.merge(guide_sleeve)
    return collar


def _build_plunger_rod_and_thumb():
    rod = CylinderGeometry(radius=0.0017, height=0.094, radial_segments=24).translate(
        0.0,
        0.0,
        -0.014,
    )
    stop_collar = CylinderGeometry(radius=0.0044, height=0.004, radial_segments=28).translate(
        0.0,
        0.0,
        0.017,
    )
    thumb_stem = CylinderGeometry(radius=0.0034, height=0.014, radial_segments=24).translate(
        0.0,
        0.0,
        0.040,
    )
    thumb_pad = CylinderGeometry(radius=0.013, height=0.005, radial_segments=32).translate(
        0.0,
        0.0,
        0.0485,
    )

    rod.merge(thumb_stem)
    rod.merge(stop_collar)
    rod.merge(thumb_pad)
    return rod


def _build_plunger_head():
    head = CylinderGeometry(radius=0.0065, height=0.008, radial_segments=32).translate(
        0.0,
        0.0,
        -0.066,
    )
    head.merge(
        CylinderGeometry(radius=0.0069, height=0.002, radial_segments=32).translate(
            0.0,
            0.0,
            -0.069,
        )
    )
    head.merge(
        CylinderGeometry(radius=0.0069, height=0.002, radial_segments=32).translate(
            0.0,
            0.0,
            -0.061,
        )
    )
    return head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_polymer = model.material("clear_polymer", rgba=(0.86, 0.90, 0.95, 0.42))
    white_plastic = model.material("white_plastic", rgba=(0.95, 0.96, 0.97, 1.0))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.30, 0.44, 0.80, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.16, 0.18, 0.20, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_geometry(_build_barrel_shell(), "base_frame_barrel_shell"),
        material=clear_polymer,
        name="barrel_shell",
    )
    base_frame.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=BARREL_REAR_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, BARREL_REAR_Z * 0.5)),
    )

    guide_module = model.part("guide_module")
    guide_module.visual(
        mesh_from_geometry(_build_guide_module(), "guide_module_body"),
        material=white_plastic,
        name="guide_body",
    )
    guide_module.inertial = Inertial.from_geometry(
        Box((0.065, 0.014, 0.020)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    moving_carriage = model.part("moving_carriage")
    moving_carriage.visual(
        mesh_from_geometry(_build_plunger_rod_and_thumb(), "moving_carriage_rod"),
        material=plunger_plastic,
        name="plunger_rod_and_thumb",
    )
    moving_carriage.visual(
        mesh_from_geometry(_build_plunger_head(), "moving_carriage_head"),
        material=seal_rubber,
        name="plunger_head",
    )
    moving_carriage.inertial = Inertial.from_geometry(
        Box((0.028, 0.028, 0.125)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.FIXED,
        parent=base_frame,
        child=guide_module,
        origin=Origin(xyz=(0.0, 0.0, BARREL_REAR_Z)),
    )
    model.articulation(
        "guide_to_moving_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_module,
        child=moving_carriage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    guide_module = object_model.get_part("guide_module")
    moving_carriage = object_model.get_part("moving_carriage")
    slide = object_model.get_articulation("guide_to_moving_carriage")

    ctx.check(
        "syringe uses three named modules",
        {part.name for part in object_model.parts}
        == {"base_frame", "guide_module", "moving_carriage"},
        details=str([part.name for part in object_model.parts]),
    )

    ctx.expect_overlap(
        guide_module,
        base_frame,
        axes="xy",
        elem_a="guide_body",
        elem_b="barrel_shell",
        min_overlap=0.016,
        name="guide module wraps around rear barrel footprint",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            moving_carriage,
            base_frame,
            axes="xy",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0025,
            name="resting plunger head stays centered in barrel",
        )
        ctx.expect_overlap(
            moving_carriage,
            base_frame,
            axes="z",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="resting plunger head remains inside barrel length",
        )

    rest_pos = ctx.part_world_position(moving_carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            moving_carriage,
            base_frame,
            axes="xy",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0025,
            name="retracted plunger head stays centered in barrel",
        )
        ctx.expect_overlap(
            moving_carriage,
            base_frame,
            axes="z",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="retracted plunger head retains insertion in barrel",
        )
        extended_pos = ctx.part_world_position(moving_carriage)

    ctx.check(
        "plunger retracts rearward along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.040,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
