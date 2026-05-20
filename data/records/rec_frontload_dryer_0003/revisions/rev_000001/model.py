from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CABINET_WIDTH = 0.98
CABINET_DEPTH = 1.18
CABINET_HEIGHT = 1.88
SHEET = 0.018
BASE_HEIGHT = 0.080

DOOR_CENTER_Z = 0.98
OPENING_RADIUS = 0.330
DOOR_OUTER_RADIUS = 0.375
DOOR_INNER_RADIUS = 0.275
DOOR_GLASS_RADIUS = 0.258
DOOR_THICKNESS = 0.070
DOOR_HINGE_X = -0.385
DOOR_JOINT_Y = CABINET_DEPTH * 0.5 + 0.035
DOOR_RING_Y_OFFSET = 0.004
HINGE_BARREL_RADIUS = 0.014

DRUM_CENTER_Y = 0.0
DRUM_LENGTH = 0.880
DRUM_OUTER_RADIUS = 0.315
DRUM_INNER_RADIUS = 0.303


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_height = height * 0.5
    return [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, depth: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [list(reversed(_circle_profile(inner_radius)))],
            depth,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _front_frame_mesh(name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(CABINET_WIDTH - (2.0 * SHEET), CABINET_HEIGHT),
            [list(reversed(_circle_profile(OPENING_RADIUS)))],
            SHEET,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_laundry_dryer")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.42, 0.48, 0.54, 0.38))
    drum_steel = model.material("drum_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.32, 0.34, 0.37, 1.0))

    front_frame_mesh = _front_frame_mesh("dryer_front_frame")
    shroud_ring_mesh = _annulus_mesh(
        "dryer_front_shroud",
        outer_radius=0.350,
        inner_radius=0.322,
        depth=0.090,
    )
    door_outer_ring_mesh = _annulus_mesh(
        "dryer_door_outer_ring",
        outer_radius=DOOR_OUTER_RADIUS,
        inner_radius=DOOR_INNER_RADIUS,
        depth=DOOR_THICKNESS,
    )
    door_inner_ring_mesh = _annulus_mesh(
        "dryer_door_inner_ring",
        outer_radius=0.332,
        inner_radius=DOOR_GLASS_RADIUS,
        depth=0.030,
    )
    drum_shell_mesh = _annulus_mesh(
        "dryer_drum_shell",
        outer_radius=DRUM_OUTER_RADIUS,
        inner_radius=DRUM_INNER_RADIUS,
        depth=DRUM_LENGTH,
    )
    drum_front_rim_mesh = _annulus_mesh(
        "dryer_drum_front_rim",
        outer_radius=DRUM_OUTER_RADIUS,
        inner_radius=0.270,
        depth=0.040,
    )
    bearing_ring_mesh = _annulus_mesh(
        "dryer_bearing_ring",
        outer_radius=0.072,
        inner_radius=0.048,
        depth=0.064,
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=stainless,
        name="base_plinth",
    )
    cabinet.visual(
        Box((SHEET, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH * 0.5 + SHEET * 0.5, 0.0, CABINET_HEIGHT * 0.5)),
        material=stainless,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SHEET, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH * 0.5 - SHEET * 0.5, 0.0, CABINET_HEIGHT * 0.5)),
        material=stainless,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - (2.0 * SHEET), SHEET, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH * 0.5 + SHEET * 0.5, CABINET_HEIGHT * 0.5)),
        material=stainless,
        name="rear_panel",
    )
    cabinet.visual(
        front_frame_mesh,
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH * 0.5 - SHEET * 0.5, CABINET_HEIGHT * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=stainless,
        name="front_frame",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - SHEET * 0.5)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.860, 0.110, 0.200)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5 - 0.055, CABINET_HEIGHT - 0.120)),
        material=stainless,
        name="control_console",
    )
    cabinet.visual(
        Box((0.720, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5 - 0.006, CABINET_HEIGHT - 0.120)),
        material=dark_trim,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.260, 0.014, 0.040)),
        origin=Origin(xyz=(-0.175, CABINET_DEPTH * 0.5 - 0.007, CABINET_HEIGHT - 0.103)),
        material=tinted_glass,
        name="display_panel",
    )
    for knob_x in (-0.010, 0.095, 0.200):
        cabinet.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(
                xyz=(knob_x, CABINET_DEPTH * 0.5 - 0.010, CABINET_HEIGHT - 0.125),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=satin_dark,
            name=f"control_knob_{int((knob_x + 0.01) * 1000):03d}",
        )
    cabinet.visual(
        shroud_ring_mesh,
        origin=Origin(
            xyz=(0.0, 0.527, DOOR_CENTER_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=black_rubber,
        name="front_shroud",
    )
    cabinet.visual(
        bearing_ring_mesh,
        origin=Origin(
            xyz=(0.0, DRUM_CENTER_Y - (DRUM_LENGTH * 0.5) - 0.142, DOOR_CENTER_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_dark,
        name="bearing_face",
    )
    cabinet.visual(
        Box((0.089, 0.036, 0.160)),
        origin=Origin(
            xyz=(DOOR_HINGE_X - 0.0585, DOOR_JOINT_Y - 0.015, DOOR_CENTER_Z + 0.195),
        ),
        material=stainless,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.089, 0.036, 0.160)),
        origin=Origin(
            xyz=(DOOR_HINGE_X - 0.0585, DOOR_JOINT_Y - 0.015, DOOR_CENTER_Z - 0.195),
        ),
        material=stainless,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.030, 0.012, 0.712)),
        origin=Origin(
            xyz=(DOOR_HINGE_X - 0.073, CABINET_DEPTH * 0.5 + 0.006, DOOR_CENTER_Z),
        ),
        material=stainless,
        name="hinge_stile",
    )
    for foot_x in (-0.360, 0.360):
        for foot_y in (-0.440, 0.440):
            cabinet.visual(
                Cylinder(radius=0.028, length=0.016),
                origin=Origin(
                    xyz=(foot_x, foot_y, 0.008),
                    rpy=(0.0, 0.0, 0.0),
                ),
                material=dark_trim,
                name=f"foot_{'r' if foot_x > 0 else 'l'}_{'f' if foot_y > 0 else 'b'}",
            )
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        drum_front_rim_mesh,
        origin=Origin(
            xyz=(0.0, DRUM_LENGTH * 0.5 - 0.020, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=stainless,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=DRUM_OUTER_RADIUS, length=0.012),
        origin=Origin(
            xyz=(0.0, -DRUM_LENGTH * 0.5 + 0.006, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=drum_steel,
        name="rear_bulkhead",
    )
    drum.visual(
        Cylinder(radius=0.048, length=0.110),
        origin=Origin(
            xyz=(0.0, -DRUM_LENGTH * 0.5 - 0.055, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_dark,
        name="axle_stub",
    )
    paddle_radius = DRUM_INNER_RADIUS - 0.015
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        drum.visual(
            Box((0.084, DRUM_LENGTH * 0.84, 0.030)),
            origin=Origin(
                xyz=(
                    paddle_radius * math.sin(angle),
                    0.0,
                    paddle_radius * math.cos(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=stainless,
            name=f"baffle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=DRUM_OUTER_RADIUS, length=DRUM_LENGTH),
        mass=26.0,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        door_outer_ring_mesh,
        origin=Origin(
            xyz=(0.385, DOOR_RING_Y_OFFSET, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=stainless,
        name="outer_ring",
    )
    door.visual(
        door_inner_ring_mesh,
        origin=Origin(
            xyz=(0.385, DOOR_RING_Y_OFFSET - 0.010, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=black_rubber,
        name="inner_bezel",
    )
    door.visual(
        Cylinder(radius=DOOR_GLASS_RADIUS, length=0.010),
        origin=Origin(
            xyz=(0.385, DOOR_RING_Y_OFFSET - 0.006, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=tinted_glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=stainless,
        name="upper_hinge_barrel",
    )
    door.visual(
        Box((0.138, 0.024, 0.056)),
        origin=Origin(xyz=(0.083, 0.0, 0.195)),
        material=stainless,
        name="upper_hinge_arm",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=stainless,
        name="lower_hinge_barrel",
    )
    door.visual(
        Box((0.138, 0.024, 0.056)),
        origin=Origin(xyz=(0.083, 0.0, -0.195)),
        material=stainless,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.034, 0.026, 0.260)),
        origin=Origin(xyz=(0.640, DOOR_RING_Y_OFFSET + 0.026, 0.0)),
        material=dark_trim,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.770, DOOR_THICKNESS, 0.770)),
        mass=9.5,
        origin=Origin(xyz=(0.385, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_CENTER_Y, DOOR_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=6.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_JOINT_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_axle = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door hinge axis is vertical",
        tuple(round(value, 6) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        f"expected vertical hinge axis, got {door_hinge.axis!r}",
    )
    ctx.check(
        "drum axle axis is depthwise",
        tuple(round(value, 6) for value in drum_axle.axis) == (0.0, 1.0, 0.0),
        f"expected drum axis along +Y, got {drum_axle.axis!r}",
    )

    ctx.expect_contact(
        door,
        cabinet,
        elem_a="upper_hinge_barrel",
        elem_b="upper_hinge_mount",
        name="upper hinge barrel contacts cabinet mount",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="lower_hinge_barrel",
        elem_b="lower_hinge_mount",
        name="lower hinge barrel contacts cabinet mount",
    )
    ctx.expect_contact(
        drum,
        cabinet,
        elem_a="axle_stub",
        elem_b="bearing_face",
        name="drum axle contacts rear bearing support",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes=("x", "z"),
        margin=0.0,
        name="drum stays within cabinet width and height",
    )
    ctx.expect_gap(
        cabinet,
        drum,
        axis="y",
        min_gap=0.10,
        max_gap=0.18,
        positive_elem="front_frame",
        negative_elem="drum_shell",
        name="drum sits behind the front opening",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.003,
            max_gap=0.012,
            positive_elem="outer_ring",
            negative_elem="front_frame",
            name="door closes just proud of the stainless front",
        )

    with ctx.pose({door_hinge: 1.30}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open door clears the cabinet")

    with ctx.pose({drum_axle: 1.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated drum clears the cabinet")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
