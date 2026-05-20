from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 1.00
FRAME_HEIGHT = 0.74
FRAME_DEPTH = 0.05
FRAME_RAIL = 0.06
OPENING_WIDTH = FRAME_WIDTH - 2.0 * FRAME_RAIL
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * FRAME_RAIL

SLAT_COUNT = 6
SLAT_CHORD = 0.065
SLAT_THICKNESS = 0.010
SUPPORT_LEN = 0.014
SUPPORT_HEIGHT = 0.035
SUPPORT_DEPTH = 0.032
SUPPORT_CLEARANCE = 0.003
BEARING_PAD_THICKNESS = SUPPORT_CLEARANCE
PIN_RADIUS = 0.0045
PIN_LENGTH = SUPPORT_LEN
SLAT_LENGTH = OPENING_WIDTH - 2.0 * (SUPPORT_LEN + SUPPORT_CLEARANCE)
SLAT_LOWER = -0.70
SLAT_UPPER = 0.70


def _slat_z_positions() -> list[float]:
    pitch = OPENING_HEIGHT / (SLAT_COUNT + 1)
    top = OPENING_HEIGHT / 2.0 - pitch
    return [top - i * pitch for i in range(SLAT_COUNT)]


def _slat_shape() -> cq.Workplane:
    blade = cq.Workplane("XY").box(SLAT_LENGTH, SLAT_THICKNESS, SLAT_CHORD)
    blade = blade.edges("|X").fillet(0.0035)
    return blade


def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: str) -> float | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return float(aabb[1][axis_index] - aabb[0][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_frame_slat_array")

    frame_finish = model.material("frame_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    slat_finish = model.material("slat_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - FRAME_RAIL / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_rail",
    )
    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + FRAME_RAIL / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + FRAME_RAIL / 2.0)),
        material=frame_finish,
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_HEIGHT / 2.0 - FRAME_RAIL / 2.0)),
        material=frame_finish,
        name="bottom_rail",
    )

    jaw_height = (SUPPORT_HEIGHT - 2.0 * PIN_RADIUS) / 2.0
    left_jaw_x = -OPENING_WIDTH / 2.0 + SUPPORT_LEN / 2.0
    right_jaw_x = OPENING_WIDTH / 2.0 - SUPPORT_LEN / 2.0
    left_pad_x = -OPENING_WIDTH / 2.0 + BEARING_PAD_THICKNESS / 2.0
    right_pad_x = OPENING_WIDTH / 2.0 - BEARING_PAD_THICKNESS / 2.0
    for index, z_pos in enumerate(_slat_z_positions(), start=1):
        upper_z = z_pos + PIN_RADIUS + jaw_height / 2.0
        lower_z = z_pos - PIN_RADIUS - jaw_height / 2.0

        frame.visual(
            Box((SUPPORT_LEN, SUPPORT_DEPTH, jaw_height)),
            origin=Origin(xyz=(left_jaw_x, 0.0, upper_z)),
            material=frame_finish,
            name=f"left_upper_support_{index}",
        )
        frame.visual(
            Box((SUPPORT_LEN, SUPPORT_DEPTH, jaw_height)),
            origin=Origin(xyz=(left_jaw_x, 0.0, lower_z)),
            material=frame_finish,
            name=f"left_lower_support_{index}",
        )
        frame.visual(
            Box((BEARING_PAD_THICKNESS, SUPPORT_DEPTH, 2.0 * PIN_RADIUS)),
            origin=Origin(xyz=(left_pad_x, 0.0, z_pos)),
            material=frame_finish,
            name=f"left_bearing_pad_{index}",
        )
        frame.visual(
            Box((SUPPORT_LEN, SUPPORT_DEPTH, jaw_height)),
            origin=Origin(xyz=(right_jaw_x, 0.0, upper_z)),
            material=frame_finish,
            name=f"right_upper_support_{index}",
        )
        frame.visual(
            Box((SUPPORT_LEN, SUPPORT_DEPTH, jaw_height)),
            origin=Origin(xyz=(right_jaw_x, 0.0, lower_z)),
            material=frame_finish,
            name=f"right_lower_support_{index}",
        )
        frame.visual(
            Box((BEARING_PAD_THICKNESS, SUPPORT_DEPTH, 2.0 * PIN_RADIUS)),
            origin=Origin(xyz=(right_pad_x, 0.0, z_pos)),
            material=frame_finish,
            name=f"right_bearing_pad_{index}",
        )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=8.0,
    )

    slat_mesh = _slat_shape()
    slat_total_length = SLAT_LENGTH + 2.0 * PIN_LENGTH
    for index, z_pos in enumerate(_slat_z_positions(), start=1):
        slat = model.part(f"slat_{index}")
        slat.visual(
            mesh_from_cadquery(slat_mesh, f"slat_{index}_shell"),
            material=slat_finish,
            name="slat_shell",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(-SLAT_LENGTH / 2.0 - PIN_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=slat_finish,
            name="left_journal",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(SLAT_LENGTH / 2.0 + PIN_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=slat_finish,
            name="right_journal",
        )
        slat.inertial = Inertial.from_geometry(
            Box((slat_total_length, SLAT_THICKNESS, SLAT_CHORD)),
            mass=0.55,
        )

        model.articulation(
            f"frame_to_slat_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=SLAT_LOWER,
                upper=SLAT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{index}") for index in range(1, SLAT_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_slat_{index}")
        for index in range(1, SLAT_COUNT + 1)
    ]

    present_parts = {part.name for part in [frame, *slats]}
    for name in ["frame", *[f"slat_{index}" for index in range(1, SLAT_COUNT + 1)]]:
        ctx.check(f"{name} present", name in present_parts, details=f"present_parts={sorted(present_parts)}")

    for index, joint in enumerate(joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"slat_{index} joint axis and limits",
            joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower == SLAT_LOWER
            and limits.upper == SLAT_UPPER,
            details=f"axis={joint.axis}, limits={limits}",
        )
        ctx.expect_contact(
            slats[index - 1],
            frame,
            name=f"slat_{index} journals seat in frame supports",
        )

    with ctx.pose({joint: SLAT_UPPER for joint in joints}):
        for index in range(1, SLAT_COUNT):
            ctx.expect_gap(
                slats[index - 1],
                slats[index],
                axis="z",
                min_gap=0.020,
                name=f"adjacent slats {index} and {index + 1} keep open-pose clearance",
            )

    active_slat = slats[2]
    neighbor_slat = slats[3]
    active_joint = joints[2]
    rest_active_span = _span(ctx.part_element_world_aabb(active_slat, elem="slat_shell"), "y")
    rest_neighbor_span = _span(ctx.part_element_world_aabb(neighbor_slat, elem="slat_shell"), "y")
    with ctx.pose({active_joint: SLAT_UPPER}):
        open_active_span = _span(ctx.part_element_world_aabb(active_slat, elem="slat_shell"), "y")
        open_neighbor_span = _span(ctx.part_element_world_aabb(neighbor_slat, elem="slat_shell"), "y")

    ctx.check(
        "one slat rotates about its own long axis",
        rest_active_span is not None
        and rest_neighbor_span is not None
        and open_active_span is not None
        and open_neighbor_span is not None
        and open_active_span > rest_active_span + 0.030
        and abs(open_neighbor_span - rest_neighbor_span) < 1e-6,
        details=(
            f"active_rest={rest_active_span}, active_open={open_active_span}, "
            f"neighbor_rest={rest_neighbor_span}, neighbor_open={open_neighbor_span}"
        ),
    )

    with ctx.pose({joints[0]: SLAT_UPPER, joints[-1]: SLAT_LOWER}):
        ctx.expect_within(
            slats[0],
            frame,
            axes="y",
            margin=0.0,
            name="top slat stays inside frame depth envelope when pitched open",
        )
        ctx.expect_within(
            slats[-1],
            frame,
            axes="y",
            margin=0.0,
            name="bottom slat stays inside frame depth envelope when pitched opposite",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
