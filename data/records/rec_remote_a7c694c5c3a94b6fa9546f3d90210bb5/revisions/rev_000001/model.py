from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_WIDTH = 0.56
HOUSING_DEPTH = 0.34
HOUSING_HEIGHT = 0.040
TOP_Z = HOUSING_HEIGHT + 0.003
SLOT_TOP_Z = TOP_Z + 0.0008
FADER_TRAVEL = 0.180
FADER_SLOT_LENGTH = 0.235
FADER_START_Y = -0.125
FADER_XS = (-0.180, -0.060, 0.060, 0.180)


def _add_fader_slot_detail(
    housing,
    *,
    x: float,
    slot_name: str,
    slot_material: Material,
    scale_material: Material,
    meter_materials: tuple[Material, Material],
    label_material: Material,
) -> None:
    """Add the fixed deck details that make one fader lane read as a real channel strip."""

    slot_width = 0.012
    slot_center_y = -0.030
    slot_z = TOP_Z + 0.0004

    housing.visual(
        Box((slot_width, FADER_SLOT_LENGTH, 0.0008)),
        origin=Origin(xyz=(x, slot_center_y, slot_z)),
        material=slot_material,
        name=slot_name,
    )
    for end_index, y in enumerate(
        (slot_center_y - FADER_SLOT_LENGTH * 0.5, slot_center_y + FADER_SLOT_LENGTH * 0.5)
    ):
        housing.visual(
            Cylinder(radius=slot_width * 0.5, length=0.0008),
            origin=Origin(xyz=(x, y, slot_z)),
            material=slot_material,
            name=f"{slot_name}_end_{end_index}",
        )

    tick_x = x + 0.033
    for tick_index, y in enumerate((-0.130, -0.095, -0.060, -0.025, 0.010, 0.045, 0.080)):
        width = 0.018 if tick_index in (0, 3, 6) else 0.011
        housing.visual(
            Box((width, 0.0022, 0.0007)),
            origin=Origin(xyz=(tick_x, y, TOP_Z + 0.00035)),
            material=scale_material,
            name=f"{slot_name}_tick_{tick_index}",
        )

    meter_x = x - 0.034
    for meter_index, y in enumerate((-0.102, -0.078, -0.054, -0.030, -0.006, 0.018)):
        material = meter_materials[1] if meter_index >= 4 else meter_materials[0]
        housing.visual(
            Box((0.011, 0.010, 0.0010)),
            origin=Origin(xyz=(meter_x, y, TOP_Z + 0.0005)),
            material=material,
            name=f"{slot_name}_meter_{meter_index}",
        )

    housing.visual(
        Box((0.070, 0.023, 0.0010)),
        origin=Origin(xyz=(x, -0.149, TOP_Z + 0.0005)),
        material=label_material,
        name=f"{slot_name}_label_window",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broadcast_fader_controller")

    charcoal = model.material("charcoal_powder_coat", rgba=(0.055, 0.058, 0.064, 1.0))
    black = model.material("matte_black_panel", rgba=(0.010, 0.011, 0.013, 1.0))
    slot_black = model.material("deep_slot_black", rgba=(0.0, 0.0, 0.0, 1.0))
    fader_plastic = model.material("fader_cap_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.75, 0.74, 1.0))
    meter_green = model.material("meter_green", rgba=(0.10, 0.95, 0.28, 1.0))
    meter_amber = model.material("meter_amber", rgba=(1.0, 0.62, 0.08, 1.0))
    label_glass = model.material("smoked_label_glass", rgba=(0.05, 0.08, 0.10, 0.65))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT * 0.5)),
        material=charcoal,
        name="outer_case",
    )
    housing.visual(
        Box((0.535, 0.315, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT + 0.0015)),
        material=black,
        name="top_inlay",
    )
    housing.visual(
        Box((0.56, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.171, 0.043)),
        material=aluminum,
        name="front_aluminum_lip",
    )

    for index, x in enumerate(FADER_XS):
        _add_fader_slot_detail(
            housing,
            x=x,
            slot_name=f"slot_{index}",
            slot_material=slot_black,
            scale_material=aluminum,
            meter_materials=(meter_green, meter_amber),
            label_material=label_glass,
        )

    for sx in (-0.250, 0.250):
        for sy in (-0.140, 0.140):
            housing.visual(
                Cylinder(radius=0.006, length=0.0012),
                origin=Origin(xyz=(sx, sy, TOP_Z + 0.0006)),
                material=aluminum,
                name=f"corner_screw_{sx:+.2f}_{sy:+.2f}",
            )

    encoder_mesh = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.018,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=36, depth=0.0007, helix_angle_deg=18.0),
            center=False,
        ),
        "endless_encoder_knob",
    )

    for index, x in enumerate(FADER_XS):
        fader = model.part(f"fader_{index}")
        fader.visual(
            Box((0.048, 0.026, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=fader_plastic,
            name="cap",
        )
        fader.visual(
            Box((0.036, 0.004, 0.002)),
            origin=Origin(xyz=(0.0, -0.0065, 0.013)),
            material=aluminum,
            name="front_grip_ridge",
        )
        fader.visual(
            Box((0.036, 0.004, 0.002)),
            origin=Origin(xyz=(0.0, 0.0065, 0.013)),
            material=aluminum,
            name="rear_grip_ridge",
        )
        fader.visual(
            Box((0.009, 0.016, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=rubber,
            name="slot_runner",
        )
        model.articulation(
            f"housing_to_fader_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, FADER_START_Y, SLOT_TOP_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=0.35,
                lower=0.0,
                upper=FADER_TRAVEL,
            ),
        )

        encoder = model.part(f"encoder_{index}")
        encoder.visual(
            encoder_mesh,
            material=rubber,
            name="knob_shell",
        )
        encoder.visual(
            Cylinder(radius=0.011, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, 0.0186)),
            material=aluminum,
            name="top_touch_disc",
        )
        model.articulation(
            f"housing_to_encoder_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=encoder,
            origin=Origin(xyz=(x, 0.125, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    ctx.check("housing_present", housing is not None, "Expected a single fixed controller housing.")
    if housing is None:
        return ctx.report()

    def _joint_kind(joint) -> str:
        return str(getattr(joint, "articulation_type", getattr(joint, "type", ""))).lower()

    for index in range(4):
        fader = object_model.get_part(f"fader_{index}")
        fader_joint = object_model.get_articulation(f"housing_to_fader_{index}")
        ctx.check(f"fader_{index}_present", fader is not None, "Expected four moving fader carriages.")
        ctx.check(
            f"fader_{index}_is_prismatic",
            fader_joint is not None and "prismatic" in _joint_kind(fader_joint),
            "Each fader carriage should slide on a prismatic joint.",
        )
        if fader is None or fader_joint is None:
            continue

        ctx.expect_gap(
            fader,
            housing,
            axis="z",
            positive_elem="cap",
            negative_elem=f"slot_{index}",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"fader_{index}_cap_sits_on_slot",
        )
        ctx.expect_overlap(
            fader,
            housing,
            axes="xy",
            elem_a="cap",
            elem_b=f"slot_{index}",
            min_overlap=0.010,
            name=f"fader_{index}_cap_covers_slot_at_front",
        )

        rest_pos = ctx.part_world_position(fader)
        with ctx.pose({fader_joint: FADER_TRAVEL}):
            ctx.expect_overlap(
                fader,
                housing,
                axes="xy",
                elem_a="cap",
                elem_b=f"slot_{index}",
                min_overlap=0.010,
                name=f"fader_{index}_cap_stays_in_slot_at_rear",
            )
            extended_pos = ctx.part_world_position(fader)
        ctx.check(
            f"fader_{index}_travels_toward_rear",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] > rest_pos[1] + FADER_TRAVEL * 0.95,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        limits = getattr(fader_joint, "motion_limits", None)
        ctx.check(
            f"fader_{index}_has_motor_fader_throw",
            limits is not None and getattr(limits, "upper", 0.0) >= 0.15,
            "A broadcast-style linear fader should have a long usable throw.",
        )

    for index in range(4):
        encoder = object_model.get_part(f"encoder_{index}")
        encoder_joint = object_model.get_articulation(f"housing_to_encoder_{index}")
        ctx.check(f"encoder_{index}_present", encoder is not None, "Expected a row of encoder knobs.")
        ctx.check(
            f"encoder_{index}_is_endless",
            encoder_joint is not None and "continuous" in _joint_kind(encoder_joint),
            "Endless encoder knobs should use continuous rotation.",
        )
        if encoder is None or encoder_joint is None:
            continue
        ctx.expect_gap(
            encoder,
            housing,
            axis="z",
            positive_elem="knob_shell",
            negative_elem="top_inlay",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"encoder_{index}_knob_seated_on_panel",
        )

    return ctx.report()


object_model = build_object_model()
