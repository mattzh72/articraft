from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_W = 0.360
HOUSING_D = 0.280
BODY_H = 0.018
TOP_PLATE_H = 0.003
TOP_Z = BODY_H + TOP_PLATE_H

PAD_COUNT = 4
PAD_PITCH = 0.052
PAD_CAP = 0.040
PAD_HOLE = 0.046
PAD_H = 0.008
PAD_TRAVEL = 0.005
PAD_GRID_Y = 0.035

KNOB_COUNT = 4
KNOB_DIAMETER = 0.028
KNOB_H = 0.019
KNOB_Y = -0.105


def _translated_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _pad_center(row: int, col: int) -> tuple[float, float]:
    x0 = -1.5 * PAD_PITCH
    y0 = PAD_GRID_Y + 1.5 * PAD_PITCH
    return (x0 + col * PAD_PITCH, y0 - row * PAD_PITCH)


def _knob_x(index: int) -> float:
    spacing = 0.072
    return (index - (KNOB_COUNT - 1) / 2.0) * spacing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_midi_pad_controller")

    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("charcoal_panel", rgba=(0.055, 0.060, 0.066, 1.0))
    model.material("dark_rubber", rgba=(0.10, 0.105, 0.11, 1.0))
    model.material("frosted_pad", rgba=(0.72, 0.74, 0.70, 1.0))
    model.material("knob_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("white_print", rgba=(0.88, 0.90, 0.86, 1.0))
    model.material("status_blue", rgba=(0.12, 0.55, 1.0, 1.0))

    pad_hole_profile = rounded_rect_profile(PAD_HOLE, PAD_HOLE, 0.004, corner_segments=5)
    pad_holes = [
        _translated_profile(pad_hole_profile, *_pad_center(row, col))
        for row in range(PAD_COUNT)
        for col in range(PAD_COUNT)
    ]

    outer_body = rounded_rect_profile(HOUSING_W, HOUSING_D, 0.018, corner_segments=10)
    body_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer_body, pad_holes, BODY_H, center=False),
        "housing_body",
    )
    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer_body, pad_holes, TOP_PLATE_H, center=False),
        "top_plate",
    )

    housing = model.part("housing")
    housing.visual(body_mesh, material="matte_black", name="housing_body")
    housing.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        material="charcoal_panel",
        name="top_plate",
    )

    # A slightly different surface area along the lower/front edge makes the
    # encoder strip read as a DJ controller control section rather than a plain
    # rectangle.
    housing.visual(
        Box((HOUSING_W - 0.035, 0.050, 0.0012)),
        origin=Origin(xyz=(0.0, KNOB_Y, TOP_Z + 0.0006)),
        material="matte_black",
        name="encoder_strip",
    )

    # Small blue status LEDs, printed legends, and encoder bushings are fixed to
    # the top plate; they give the otherwise flat face a scale and product feel.
    for col in range(PAD_COUNT):
        x, y = _pad_center(0, col)
        housing.visual(
            Box((0.018, 0.003, 0.0007)),
            origin=Origin(xyz=(x, y + 0.020, TOP_Z + 0.00015)),
            material="status_blue",
            name=f"led_{col}",
        )

    for i in range(KNOB_COUNT):
        x = _knob_x(i)
        bushing_name = "encoder_bushing_1" if i == 1 else f"encoder_bushing_{i}"
        housing.visual(
            Cylinder(radius=0.018, length=0.0012),
            origin=Origin(xyz=(x, KNOB_Y, TOP_Z - 0.0006)),
            material="dark_rubber",
            name=bushing_name,
        )
        housing.visual(
            Box((0.026, 0.002, 0.0007)),
            origin=Origin(xyz=(x, KNOB_Y - 0.026, TOP_Z + 0.00035)),
            material="white_print",
            name=f"label_{i}",
        )

    pad_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(PAD_CAP, PAD_CAP, 0.005, corner_segments=5),
            PAD_H,
        ),
        "trigger_pad_cap",
    )

    for row in range(PAD_COUNT):
        for col in range(PAD_COUNT):
            x, y = _pad_center(row, col)
            pad = model.part(f"pad_{row}_{col}")
            pad.visual(
                pad_cap_mesh,
                material="frosted_pad",
                name="pad_cap",
            )
            pad.visual(
                Box((0.030, 0.030, 0.006)),
                origin=Origin(xyz=(0.0, 0.0, -0.003)),
                material="dark_rubber",
                name="guide_plunger",
            )
            model.articulation(
                f"housing_to_pad_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=pad,
                origin=Origin(xyz=(x, y, TOP_Z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=0.08,
                    lower=-PAD_TRAVEL,
                    upper=0.0,
                ),
                motion_properties=MotionProperties(damping=0.35, friction=0.08),
            )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_H,
            body_style="cylindrical",
            edge_radius=0.0009,
            grip=KnobGrip(style="knurled", count=32, depth=0.0007, helix_angle_deg=18.0),
            center=False,
        ),
        "encoder_knob",
    )

    for i in range(KNOB_COUNT):
        x = _knob_x(i)
        encoder = model.part(f"encoder_{i}")
        encoder.visual(knob_mesh, material="knob_grey", name="knob_body")
        encoder.visual(
            Box((0.012, 0.0022, 0.0010)),
            origin=Origin(xyz=(0.0045, 0.0, KNOB_H + 0.0005)),
            material="white_print",
            name="indicator_line",
        )
        model.articulation(
            f"housing_to_encoder_{i}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=encoder,
            origin=Origin(xyz=(x, KNOB_Y, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.8, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pad_joints = [
        object_model.get_articulation(f"housing_to_pad_{row}_{col}")
        for row in range(PAD_COUNT)
        for col in range(PAD_COUNT)
    ]
    encoder_joints = [
        object_model.get_articulation(f"housing_to_encoder_{i}") for i in range(KNOB_COUNT)
    ]

    ctx.check(
        "has 16 prismatic trigger pads",
        len(pad_joints) == 16
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in pad_joints),
        details=f"pad_joints={pad_joints}",
    )
    ctx.check(
        "has continuous rotary encoders",
        len(encoder_joints) == KNOB_COUNT
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in encoder_joints),
        details=f"encoder_joints={encoder_joints}",
    )

    sample_pad = object_model.get_part("pad_1_1")
    sample_pad_joint = object_model.get_articulation("housing_to_pad_1_1")
    housing = object_model.get_part("housing")

    ctx.expect_within(
        sample_pad,
        housing,
        axes="xy",
        elem_a="pad_cap",
        elem_b="top_plate",
        margin=0.0,
        name="sample pad sits within the top-plate footprint",
    )
    ctx.expect_gap(
        sample_pad,
        housing,
        axis="z",
        positive_elem="pad_cap",
        negative_elem="top_plate",
        min_gap=-0.0002,
        max_gap=0.0002,
        name="sample pad cap starts level with the deck",
    )

    rest_pos = ctx.part_world_position(sample_pad)
    with ctx.pose({sample_pad_joint: -PAD_TRAVEL}):
        pressed_pos = ctx.part_world_position(sample_pad)
    ctx.check(
        "sample pad presses downward",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - PAD_TRAVEL * 0.8,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    sample_encoder = object_model.get_part("encoder_1")
    ctx.expect_gap(
        sample_encoder,
        housing,
        axis="z",
        positive_elem="knob_body",
        negative_elem="encoder_bushing_1",
        min_gap=-0.0002,
        max_gap=0.0002,
        name="sample encoder sits on its bushing",
    )

    return ctx.report()


object_model = build_object_model()
