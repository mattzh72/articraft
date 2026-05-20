from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="milling_machine_vise", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.34, 0.37, 0.41, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    base_length = 0.34
    base_width = 0.18
    base_thickness = 0.022

    way_length = 0.26
    way_width = 0.032
    way_height = 0.012
    way_offset_y = 0.048

    fixed_body_length = 0.055
    fixed_body_width = 0.164
    fixed_body_height = 0.052
    fixed_body_center_x = 0.1225

    jaw_plate_thickness = 0.008
    fixed_plate_width = 0.144
    fixed_plate_height = 0.052
    fixed_plate_center_x = 0.091

    base = model.part("base")
    base.visual(
        Box((base_length, base_width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((way_length, way_width, way_height)),
        origin=Origin(xyz=(0.0, way_offset_y, base_thickness + way_height / 2.0)),
        material=ground_steel,
        name="way_front",
    )
    base.visual(
        Box((way_length, way_width, way_height)),
        origin=Origin(xyz=(0.0, -way_offset_y, base_thickness + way_height / 2.0)),
        material=ground_steel,
        name="way_rear",
    )
    base.visual(
        Box((0.27, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.002)),
        material=ground_steel,
        name="bed_strip",
    )
    base.visual(
        Box((fixed_body_length, fixed_body_width, fixed_body_height)),
        origin=Origin(
            xyz=(fixed_body_center_x, 0.0, base_thickness + fixed_body_height / 2.0)
        ),
        material=cast_iron,
        name="fixed_jaw_body",
    )
    base.visual(
        Box((jaw_plate_thickness, fixed_plate_width, fixed_plate_height)),
        origin=Origin(
            xyz=(fixed_plate_center_x, 0.0, base_thickness + 0.010 + fixed_plate_height / 2.0)
        ),
        material=hardened_steel,
        name="fixed_jaw_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, 0.08)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    slide_origin_x = 0.040
    slide_origin_y = way_offset_y
    slide_origin_z = base_thickness + way_height
    pad_length = 0.080
    pad_width = 0.030
    pad_height = 0.010
    pad_center_x = -0.002

    movable_body_length = 0.055
    movable_body_width = 0.164
    movable_body_height = 0.050
    movable_body_center_x = -0.005

    movable_plate_width = 0.140
    movable_plate_height = 0.048
    movable_plate_center_x = 0.0265

    movable_jaw = model.part("movable_jaw")
    movable_jaw.visual(
        Box((pad_length, pad_width, pad_height)),
        origin=Origin(
            xyz=(pad_center_x, 0.0, pad_height / 2.0)
        ),
        material=ground_steel,
        name="slide_pad_front",
    )
    movable_jaw.visual(
        Box((pad_length, pad_width, pad_height)),
        origin=Origin(
            xyz=(pad_center_x, -2.0 * way_offset_y, pad_height / 2.0)
        ),
        material=ground_steel,
        name="slide_pad_rear",
    )
    movable_jaw.visual(
        Box((movable_body_length, movable_body_width, movable_body_height)),
        origin=Origin(
            xyz=(
                movable_body_center_x,
                -slide_origin_y,
                pad_height + movable_body_height / 2.0,
            )
        ),
        material=cast_iron,
        name="movable_jaw_body",
    )
    movable_jaw.visual(
        Box((jaw_plate_thickness, movable_plate_width, movable_plate_height)),
        origin=Origin(
            xyz=(
                movable_plate_center_x,
                -slide_origin_y,
                base_thickness + 0.010 + movable_plate_height / 2.0 - slide_origin_z,
            )
        ),
        material=hardened_steel,
        name="movable_jaw_plate",
    )
    movable_jaw.inertial = Inertial.from_geometry(
        Box((0.09, movable_body_width, 0.08)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -slide_origin_y, 0.04 - slide_origin_z)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=movable_jaw,
        origin=Origin(xyz=(slide_origin_x, slide_origin_y, slide_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.15,
            lower=-0.10,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    movable_jaw = object_model.get_part("movable_jaw")
    jaw_slide = object_model.get_articulation("jaw_slide")

    fixed_jaw_plate = base.get_visual("fixed_jaw_plate")
    fixed_jaw_body = base.get_visual("fixed_jaw_body")
    way_front = base.get_visual("way_front")
    way_rear = base.get_visual("way_rear")
    base_plate = base.get_visual("base_plate")

    movable_jaw_plate = movable_jaw.get_visual("movable_jaw_plate")
    movable_jaw_body = movable_jaw.get_visual("movable_jaw_body")
    slide_pad_front = movable_jaw.get_visual("slide_pad_front")
    slide_pad_rear = movable_jaw.get_visual("slide_pad_rear")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(movable_jaw, base, elem_a=slide_pad_front, elem_b=way_front)
    ctx.expect_contact(movable_jaw, base, elem_a=slide_pad_rear, elem_b=way_rear)
    ctx.expect_contact(base, base, elem_a=fixed_jaw_plate, elem_b=fixed_jaw_body)
    ctx.expect_contact(movable_jaw, movable_jaw, elem_a=movable_jaw_plate, elem_b=movable_jaw_body)
    ctx.expect_gap(
        movable_jaw,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=slide_pad_front,
        negative_elem=way_front,
    )
    ctx.expect_gap(
        movable_jaw,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=slide_pad_rear,
        negative_elem=way_rear,
    )
    ctx.expect_within(
        movable_jaw,
        base,
        axes="yz",
        inner_elem=movable_jaw_plate,
        outer_elem=fixed_jaw_plate,
    )
    ctx.expect_overlap(
        base,
        movable_jaw,
        axes="yz",
        min_overlap=0.045,
        elem_a=fixed_jaw_plate,
        elem_b=movable_jaw_plate,
    )
    ctx.expect_within(
        movable_jaw,
        base,
        axes="xy",
        inner_elem=movable_jaw_plate,
        outer_elem=base_plate,
    )
    ctx.expect_gap(
        base,
        movable_jaw,
        axis="x",
        max_gap=0.025,
        max_penetration=0.0,
        positive_elem=fixed_jaw_plate,
        negative_elem=movable_jaw_plate,
        name="closed_jaw_gap_is_small",
    )

    with ctx.pose({jaw_slide: -0.08}):
        ctx.expect_contact(movable_jaw, base, elem_a=slide_pad_front, elem_b=way_front)
        ctx.expect_contact(movable_jaw, base, elem_a=slide_pad_rear, elem_b=way_rear)
        ctx.expect_overlap(
            base,
            movable_jaw,
            axes="yz",
            min_overlap=0.045,
            elem_a=fixed_jaw_plate,
            elem_b=movable_jaw_plate,
        )
        ctx.expect_gap(
            movable_jaw,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=slide_pad_front,
            negative_elem=way_front,
        )
        ctx.expect_gap(
            movable_jaw,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=slide_pad_rear,
            negative_elem=way_rear,
        )
        ctx.expect_gap(
            base,
            movable_jaw,
            axis="x",
            min_gap=0.095,
            positive_elem=fixed_jaw_plate,
            negative_elem=movable_jaw_plate,
            name="open_jaw_gap_is_wide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
