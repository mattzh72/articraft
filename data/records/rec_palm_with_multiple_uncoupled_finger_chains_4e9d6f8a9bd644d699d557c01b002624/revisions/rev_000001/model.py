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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


FINGER_ORDER = ("thumb", "index", "middle", "ring", "little")


def _add_finger_link(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    knuckle_radius: float,
    material,
    add_tip: bool = False,
) -> None:
    part.visual(
        Cylinder(radius=knuckle_radius, length=width + 0.004),
        origin=Origin(
            xyz=(knuckle_radius, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="joint_barrel",
    )
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(xyz=(knuckle_radius + (length * 0.5), 0.0, 0.0)),
        material=material,
        name="link_body",
    )
    if add_tip:
        part.visual(
            Sphere(radius=thickness * 0.45),
            origin=Origin(xyz=(knuckle_radius + length, 0.0, 0.0)),
            material=material,
            name="tip_pad",
        )


def _add_finger_chain(model: ArticulatedObject, rail, spec: dict[str, object], material) -> None:
    finger = spec["name"]
    proximal = model.part(f"{finger}_proximal")
    middle = model.part(f"{finger}_middle")
    distal = model.part(f"{finger}_distal")

    _add_finger_link(
        proximal,
        length=spec["proximal_length"],
        width=spec["width"],
        thickness=spec["thickness"],
        knuckle_radius=spec["knuckle_radius"],
        material=material,
    )
    _add_finger_link(
        middle,
        length=spec["middle_length"],
        width=spec["width"] * 0.92,
        thickness=spec["thickness"] * 0.92,
        knuckle_radius=spec["knuckle_radius"] * 0.90,
        material=material,
    )
    _add_finger_link(
        distal,
        length=spec["distal_length"],
        width=spec["width"] * 0.84,
        thickness=spec["thickness"] * 0.84,
        knuckle_radius=spec["knuckle_radius"] * 0.82,
        material=material,
        add_tip=True,
    )

    model.articulation(
        f"rail_to_{finger}_base",
        ArticulationType.REVOLUTE,
        parent=rail,
        child=proximal,
        origin=Origin(
            xyz=(spec["root_x"], spec["y"], spec["z"]),
            rpy=(0.0, 0.0, spec["yaw"]),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=spec["base_lower"],
            upper=spec["base_upper"],
        ),
    )
    model.articulation(
        f"{finger}_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(spec["knuckle_radius"] + spec["proximal_length"], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=0.0,
            upper=spec["middle_upper"],
        ),
    )
    model.articulation(
        f"{finger}_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(
            xyz=((spec["knuckle_radius"] * 0.90) + spec["middle_length"], 0.0, 0.0)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=spec["distal_upper"],
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="research_palm_hand")

    housing = model.material("housing_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    guide = model.material("guide_black", rgba=(0.10, 0.11, 0.12, 1.0))
    carriage = model.material("carriage_grey", rgba=(0.43, 0.45, 0.48, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.62, 0.65, 0.69, 1.0))
    finger_shell = model.material("finger_shell", rgba=(0.70, 0.72, 0.74, 1.0))
    fingertip = model.material("fingertip_dark", rgba=(0.24, 0.25, 0.27, 1.0))

    palm_block = model.part("palm_block")
    palm_block.visual(
        Box((0.070, 0.090, 0.028)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=housing,
        name="palm_core",
    )
    palm_block.visual(
        Box((0.030, 0.056, 0.024)),
        origin=Origin(xyz=(-0.045, 0.000, 0.012)),
        material=housing,
        name="wrist_stub",
    )
    palm_block.visual(
        Box((0.050, 0.012, 0.0075)),
        origin=Origin(xyz=(0.003, 0.024, 0.03125)),
        material=guide,
        name="left_slide_way",
    )
    palm_block.visual(
        Box((0.050, 0.012, 0.0075)),
        origin=Origin(xyz=(0.003, -0.024, 0.03125)),
        material=guide,
        name="right_slide_way",
    )
    palm_block.visual(
        Box((0.020, 0.040, 0.010)),
        origin=Origin(xyz=(0.025, 0.000, 0.019)),
        material=guide,
        name="front_bearing_saddle",
    )

    slider_stage = model.part("slider_stage")
    slider_stage.visual(
        Box((0.045, 0.072, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=carriage,
        name="stage_plate",
    )
    slider_stage.visual(
        Box((0.016, 0.062, 0.020)),
        origin=Origin(xyz=(0.016, 0.000, 0.021)),
        material=carriage,
        name="rail_riser",
    )
    slider_stage.visual(
        Box((0.010, 0.040, 0.016)),
        origin=Origin(xyz=(-0.015, 0.000, 0.013)),
        material=guide,
        name="rear_encoder_block",
    )

    model.articulation(
        "palm_to_slider_stage",
        ArticulationType.PRISMATIC,
        parent=palm_block,
        child=slider_stage,
        origin=Origin(xyz=(0.000, 0.000, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    finger_rail = model.part("finger_rail")
    finger_rail.visual(
        Box((0.016, 0.126, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.007)),
        material=rail_metal,
        name="rail_body",
    )
    finger_rail.visual(
        Box((0.012, 0.022, 0.020)),
        origin=Origin(xyz=(-0.001, 0.045, 0.010)),
        material=guide,
        name="left_end_cap",
    )
    finger_rail.visual(
        Box((0.012, 0.022, 0.020)),
        origin=Origin(xyz=(-0.001, -0.045, 0.010)),
        material=guide,
        name="right_end_cap",
    )
    finger_rail.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.005, 0.034, 0.005)),
        material=guide,
        name="index_mount",
    )
    finger_rail.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.005, 0.012, 0.005)),
        material=guide,
        name="middle_mount",
    )
    finger_rail.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.005, -0.012, 0.005)),
        material=guide,
        name="ring_mount",
    )
    finger_rail.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.005, -0.034, 0.005)),
        material=guide,
        name="little_mount",
    )
    finger_rail.visual(
        Box((0.016, 0.014, 0.009)),
        origin=Origin(xyz=(0.008, -0.057, 0.0105)),
        material=guide,
        name="thumb_mount",
    )

    model.articulation(
        "slider_stage_to_finger_rail",
        ArticulationType.FIXED,
        parent=slider_stage,
        child=finger_rail,
        origin=Origin(xyz=(0.024, 0.000, 0.031)),
    )

    finger_specs = (
        {
            "name": "thumb",
            "y": -0.057,
            "z": 0.002,
            "yaw": -0.72,
            "proximal_length": 0.036,
            "middle_length": 0.026,
            "distal_length": 0.020,
            "width": 0.018,
            "thickness": 0.016,
            "knuckle_radius": 0.0065,
            "root_x": 0.016,
            "base_lower": -0.25,
            "base_upper": 1.10,
            "middle_upper": 1.20,
            "distal_upper": 1.00,
        },
        {
            "name": "index",
            "y": 0.034,
            "z": 0.002,
            "yaw": 0.12,
            "proximal_length": 0.046,
            "middle_length": 0.028,
            "distal_length": 0.022,
            "width": 0.015,
            "thickness": 0.014,
            "knuckle_radius": 0.0055,
            "root_x": 0.010,
            "base_lower": 0.0,
            "base_upper": 1.28,
            "middle_upper": 1.45,
            "distal_upper": 1.15,
        },
        {
            "name": "middle",
            "y": 0.012,
            "z": 0.002,
            "yaw": 0.03,
            "proximal_length": 0.051,
            "middle_length": 0.032,
            "distal_length": 0.024,
            "width": 0.016,
            "thickness": 0.015,
            "knuckle_radius": 0.0058,
            "root_x": 0.010,
            "base_lower": 0.0,
            "base_upper": 1.30,
            "middle_upper": 1.48,
            "distal_upper": 1.18,
        },
        {
            "name": "ring",
            "y": -0.012,
            "z": 0.002,
            "yaw": -0.04,
            "proximal_length": 0.048,
            "middle_length": 0.030,
            "distal_length": 0.022,
            "width": 0.015,
            "thickness": 0.014,
            "knuckle_radius": 0.0055,
            "root_x": 0.010,
            "base_lower": 0.0,
            "base_upper": 1.28,
            "middle_upper": 1.44,
            "distal_upper": 1.12,
        },
        {
            "name": "little",
            "y": -0.034,
            "z": 0.002,
            "yaw": -0.15,
            "proximal_length": 0.040,
            "middle_length": 0.024,
            "distal_length": 0.019,
            "width": 0.013,
            "thickness": 0.012,
            "knuckle_radius": 0.0048,
            "root_x": 0.010,
            "base_lower": 0.0,
            "base_upper": 1.24,
            "middle_upper": 1.38,
            "distal_upper": 1.08,
        },
    )

    for spec in finger_specs:
        _add_finger_chain(model, finger_rail, spec, finger_shell)
        distal = model.get_part(f"{spec['name']}_distal")
        distal.visual(
            Box((spec["distal_length"] * 0.35, spec["width"] * 0.85, spec["thickness"] * 0.45)),
            origin=Origin(
                xyz=(
                    (spec["knuckle_radius"] * 0.82) + spec["distal_length"] * 0.88,
                    0.0,
                    -(spec["thickness"] * 0.16),
                )
            ),
            material=fingertip,
            name="tactile_pad",
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
    palm_block = object_model.get_part("palm_block")
    slider_stage = object_model.get_part("slider_stage")
    finger_rail = object_model.get_part("finger_rail")
    palm_slide = object_model.get_articulation("palm_to_slider_stage")
    index_base = object_model.get_articulation("rail_to_index_base")

    for finger in FINGER_ORDER:
        ctx.check(
            f"{finger} chain parts exist",
            object_model.get_part(f"{finger}_proximal") is not None
            and object_model.get_part(f"{finger}_middle") is not None
            and object_model.get_part(f"{finger}_distal") is not None,
        )
        ctx.expect_origin_gap(
            object_model.get_part(f"{finger}_proximal"),
            finger_rail,
            axis="x",
            min_gap=0.009,
            max_gap=0.017,
            name=f"{finger} base joint sits just ahead of the rail spine",
        )

    ctx.expect_gap(
        slider_stage,
        palm_block,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="slider stage rides on palm guides",
    )
    ctx.expect_overlap(
        slider_stage,
        palm_block,
        axes="xy",
        min_overlap=0.030,
        name="slider stage remains carried by the palm block",
    )

    rail_rest = ctx.part_world_position(finger_rail)
    middle_rest = ctx.part_world_position(object_model.get_part("middle_distal"))
    index_rest = ctx.part_world_position(object_model.get_part("index_distal"))
    with ctx.pose({palm_slide: 0.018}):
        ctx.expect_overlap(
            slider_stage,
            palm_block,
            axes="x",
            min_overlap=0.024,
            name="extended stage retains insertion on the palm block",
        )
        rail_extended = ctx.part_world_position(finger_rail)
    ctx.check(
        "finger rail slides forward over the palm block",
        rail_rest is not None
        and rail_extended is not None
        and rail_extended[0] > rail_rest[0] + 0.012,
        details=f"rest={rail_rest}, extended={rail_extended}",
    )

    with ctx.pose({index_base: 0.85}):
        index_curled = ctx.part_world_position(object_model.get_part("index_distal"))
        middle_steady = ctx.part_world_position(object_model.get_part("middle_distal"))
    ctx.check(
        "index base joint curls its own chain downward",
        index_rest is not None
        and index_curled is not None
        and index_curled[2] < index_rest[2] - 0.020,
        details=f"rest={index_rest}, curled={index_curled}",
    )
    ctx.check(
        "index base articulation does not drag the neighboring middle chain",
        middle_rest is not None
        and middle_steady is not None
        and abs(middle_steady[0] - middle_rest[0]) < 1e-6
        and abs(middle_steady[1] - middle_rest[1]) < 1e-6
        and abs(middle_steady[2] - middle_rest[2]) < 1e-6,
        details=f"rest={middle_rest}, posed={middle_steady}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
