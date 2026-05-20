from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalanced_easel")

    wood = Material("wood", rgba=(0.65, 0.45, 0.25, 1.0))
    metal = Material("metal", rgba=(0.7, 0.7, 0.7, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.3, 0.3, 0.3, 1.0))

    frame = model.part("frame")
    
    # Base
    frame.visual(Box((0.08, 0.8, 0.08)), origin=Origin(xyz=(-0.35, 0, 0.04)), material=wood, name="left_foot")
    frame.visual(Box((0.08, 0.8, 0.08)), origin=Origin(xyz=(0.35, 0, 0.04)), material=wood, name="right_foot")
    frame.visual(Box((0.62, 0.08, 0.08)), origin=Origin(xyz=(0, 0, 0.04)), material=wood, name="base_crossbar")

    # Uprights
    frame.visual(Box((0.06, 0.1, 2.0)), origin=Origin(xyz=(-0.25, 0, 1.04)), material=wood, name="left_upright")
    frame.visual(Box((0.06, 0.1, 2.0)), origin=Origin(xyz=(0.25, 0, 1.04)), material=wood, name="right_upright")
    frame.visual(Box((0.56, 0.1, 0.06)), origin=Origin(xyz=(0, 0, 2.01)), material=wood, name="top_crossbar")

    # Front Rails (T-slot style)
    frame.visual(Box((0.02, 0.02, 1.8)), origin=Origin(xyz=(-0.25, 0.06, 1.04)), material=metal, name="left_front_rail_base")
    frame.visual(Box((0.04, 0.02, 1.8)), origin=Origin(xyz=(-0.25, 0.08, 1.04)), material=metal, name="left_front_rail_top")
    frame.visual(Box((0.02, 0.02, 1.8)), origin=Origin(xyz=(0.25, 0.06, 1.04)), material=metal, name="right_front_rail_base")
    frame.visual(Box((0.04, 0.02, 1.8)), origin=Origin(xyz=(0.25, 0.08, 1.04)), material=metal, name="right_front_rail_top")

    # Back Rails
    frame.visual(Box((0.02, 0.02, 1.8)), origin=Origin(xyz=(-0.25, -0.06, 1.04)), material=metal, name="left_back_rail_base")
    frame.visual(Box((0.04, 0.02, 1.8)), origin=Origin(xyz=(-0.25, -0.08, 1.04)), material=metal, name="left_back_rail_top")
    frame.visual(Box((0.02, 0.02, 1.8)), origin=Origin(xyz=(0.25, -0.06, 1.04)), material=metal, name="right_back_rail_base")
    frame.visual(Box((0.04, 0.02, 1.8)), origin=Origin(xyz=(0.25, -0.08, 1.04)), material=metal, name="right_back_rail_top")

    # Pulleys and Brackets
    frame.visual(Box((0.01, 0.06, 0.15)), origin=Origin(xyz=(-0.28, 0, 2.085)), material=metal, name="left_pulley_bracket_outer")
    frame.visual(Box((0.01, 0.06, 0.15)), origin=Origin(xyz=(-0.22, 0, 2.085)), material=metal, name="left_pulley_bracket_inner")
    frame.visual(Cylinder(radius=0.01, length=0.06), origin=Origin(xyz=(-0.25, 0, 2.15), rpy=(0, math.pi/2, 0)), material=metal, name="left_pulley_axle")
    frame.visual(Cylinder(radius=0.10, length=0.02), origin=Origin(xyz=(-0.25, 0, 2.15), rpy=(0, math.pi/2, 0)), material=dark_metal, name="left_pulley")

    frame.visual(Box((0.01, 0.06, 0.15)), origin=Origin(xyz=(0.22, 0, 2.085)), material=metal, name="right_pulley_bracket_inner")
    frame.visual(Box((0.01, 0.06, 0.15)), origin=Origin(xyz=(0.28, 0, 2.085)), material=metal, name="right_pulley_bracket_outer")
    frame.visual(Cylinder(radius=0.01, length=0.06), origin=Origin(xyz=(0.25, 0, 2.15), rpy=(0, math.pi/2, 0)), material=metal, name="right_pulley_axle")
    frame.visual(Cylinder(radius=0.10, length=0.02), origin=Origin(xyz=(0.25, 0, 2.15), rpy=(0, math.pi/2, 0)), material=dark_metal, name="right_pulley")

    # Cradle
    cradle = model.part("cradle")
    cradle.visual(Box((0.6, 0.02, 0.2)), origin=Origin(xyz=(0, 0.10, 0)), material=wood, name="back_plate")

    # Left guide
    cradle.visual(Box((0.02, 0.04, 0.2)), origin=Origin(xyz=(-0.28, 0.08, 0)), material=wood, name="left_outer_flange")
    cradle.visual(Box((0.02, 0.04, 0.2)), origin=Origin(xyz=(-0.22, 0.08, 0)), material=wood, name="left_inner_flange")
    cradle.visual(Box((0.01, 0.02, 0.2)), origin=Origin(xyz=(-0.265, 0.06, 0)), material=wood, name="left_outer_lip")
    cradle.visual(Box((0.01, 0.02, 0.2)), origin=Origin(xyz=(-0.235, 0.06, 0)), material=wood, name="left_inner_lip")

    # Right guide
    cradle.visual(Box((0.02, 0.04, 0.2)), origin=Origin(xyz=(0.22, 0.08, 0)), material=wood, name="right_inner_flange")
    cradle.visual(Box((0.02, 0.04, 0.2)), origin=Origin(xyz=(0.28, 0.08, 0)), material=wood, name="right_outer_flange")
    cradle.visual(Box((0.01, 0.02, 0.2)), origin=Origin(xyz=(0.235, 0.06, 0)), material=wood, name="right_inner_lip")
    cradle.visual(Box((0.01, 0.02, 0.2)), origin=Origin(xyz=(0.265, 0.06, 0)), material=wood, name="right_outer_lip")

    # Shelf
    cradle.visual(Box((0.8, 0.15, 0.04)), origin=Origin(xyz=(0, 0.175, -0.08)), material=wood, name="shelf_base")
    cradle.visual(Box((0.8, 0.02, 0.04)), origin=Origin(xyz=(0, 0.24, -0.04)), material=wood, name="shelf_lip")

    # Spine
    cradle.visual(Box((0.08, 0.02, 0.8)), origin=Origin(xyz=(0, 0.12, 0.3)), material=wood, name="spine")

    model.articulation(
        "cradle_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0, 0, 1.04)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-0.7, upper=0.7)
    )

    # Top Clamp
    top_clamp = model.part("top_clamp")
    top_clamp.visual(Box((0.12, 0.02, 0.08)), origin=Origin(xyz=(0, 0.14, 0)), material=wood, name="clamp_front")
    top_clamp.visual(Box((0.12, 0.02, 0.08)), origin=Origin(xyz=(0, 0.10, 0)), material=wood, name="clamp_back")
    top_clamp.visual(Box((0.02, 0.06, 0.08)), origin=Origin(xyz=(-0.05, 0.12, 0)), material=wood, name="clamp_left")
    top_clamp.visual(Box((0.02, 0.06, 0.08)), origin=Origin(xyz=(0.05, 0.12, 0)), material=wood, name="clamp_right")
    top_clamp.visual(Box((0.2, 0.06, 0.02)), origin=Origin(xyz=(0, 0.17, -0.05)), material=wood, name="clamp_overhang")
    top_clamp.visual(Box((0.2, 0.02, 0.04)), origin=Origin(xyz=(0, 0.19, -0.08)), material=wood, name="clamp_lip")

    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=cradle,
        child=top_clamp,
        origin=Origin(xyz=(0, 0, 0.3)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-0.2, upper=0.35)
    )

    # Counterweight
    counterweight = model.part("counterweight")
    counterweight.visual(Box((0.6, 0.02, 0.15)), origin=Origin(xyz=(0, -0.10, 0)), material=metal, name="front_plate")

    counterweight.visual(Box((0.02, 0.04, 0.15)), origin=Origin(xyz=(-0.28, -0.08, 0)), material=metal, name="cw_left_outer_flange")
    counterweight.visual(Box((0.02, 0.04, 0.15)), origin=Origin(xyz=(-0.22, -0.08, 0)), material=metal, name="cw_left_inner_flange")
    counterweight.visual(Box((0.01, 0.02, 0.15)), origin=Origin(xyz=(-0.265, -0.06, 0)), material=metal, name="cw_left_outer_lip")
    counterweight.visual(Box((0.01, 0.02, 0.15)), origin=Origin(xyz=(-0.235, -0.06, 0)), material=metal, name="cw_left_inner_lip")

    counterweight.visual(Box((0.02, 0.04, 0.15)), origin=Origin(xyz=(0.22, -0.08, 0)), material=metal, name="cw_right_inner_flange")
    counterweight.visual(Box((0.02, 0.04, 0.15)), origin=Origin(xyz=(0.28, -0.08, 0)), material=metal, name="cw_right_outer_flange")
    counterweight.visual(Box((0.01, 0.02, 0.15)), origin=Origin(xyz=(0.235, -0.06, 0)), material=metal, name="cw_right_inner_lip")
    counterweight.visual(Box((0.01, 0.02, 0.15)), origin=Origin(xyz=(0.265, -0.06, 0)), material=metal, name="cw_right_outer_lip")

    counterweight.visual(Box((0.4, 0.08, 0.2)), origin=Origin(xyz=(0, -0.15, 0)), material=dark_metal, name="weight_block")

    model.articulation(
        "counterweight_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(0, 0, 1.04)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-0.7, upper=0.7),
        mimic=Mimic(joint="cradle_slide", multiplier=-1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # We allow overlap between the sliders and rails because they wrap tightly
    # and might have exact surface contact or slight embedding depending on float precision.
    ctx.allow_overlap("cradle", "frame", reason="Cradle wraps around the front rails to slide.")
    ctx.allow_overlap("counterweight", "frame", reason="Counterweight wraps around the back rails to slide.")
    ctx.allow_overlap("top_clamp", "cradle", reason="Top clamp wraps around the cradle spine to slide.")

    return ctx.report()

object_model = build_object_model()
