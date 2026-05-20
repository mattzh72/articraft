from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_turntable")

    walnut = Material("dark_walnut", rgba=(0.18, 0.11, 0.055, 1.0))
    satin_black = Material("satin_black", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    steel = Material("polished_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_grey = Material("dark_grey", rgba=(0.08, 0.085, 0.09, 1.0))
    white = Material("stylus_white", rgba=(0.92, 0.9, 0.84, 1.0))

    # Root plinth: a real-size rectangular turntable base with a flush top deck,
    # stationary tonearm bearing, and a front-edge cueing-lever saddle.
    plinth = model.part("plinth")
    plinth.visual(
        Box((0.50, 0.38, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=walnut,
        name="wood_base",
    )
    plinth.visual(
        Box((0.46, 0.34, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=satin_black,
        name="top_plate",
    )
    plinth.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(0.170, 0.110, 0.084)),
        material=aluminum,
        name="arm_pedestal",
    )
    plinth.visual(
        Box((0.070, 0.038, 0.019)),
        origin=Origin(xyz=(0.110, -0.176, 0.0805)),
        material=dark_grey,
        name="cue_saddle",
    )

    # The platter rotates continuously on a central vertical axle.  A small
    # bearing hub touches the plinth deck, the aluminum platter rests above it,
    # and a rubber mat plus spindle make it read as DJ hardware rather than a
    # generic disk.
    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="bearing_hub",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=aluminum,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.125, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=rubber,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.037),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=steel,
        name="center_spindle",
    )

    model.articulation(
        "platter_axle",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.075, 0.025, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )

    # Vertical tonearm pivot assembly.  This is the rotating gimbal carrier
    # sitting on the fixed plinth pedestal.
    pivot = model.part("arm_pivot")
    pivot.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=aluminum,
        name="lower_collar",
    )
    pivot.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=steel,
        name="pivot_post",
    )
    pivot.visual(
        Box((0.050, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_grey,
        name="gimbal_block",
    )

    model.articulation(
        "arm_sweep",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=pivot,
        origin=Origin(xyz=(0.170, 0.110, 0.097)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-0.75, upper=0.75),
    )

    # Tonearm child frame sits at the horizontal arm-height / anti-skate bearing.
    # Its local +X points from the pivot toward the record groove area.
    arm_yaw = -2.447
    tonearm = model.part("tonearm")
    tonearm.visual(
        Box((0.040, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_grey,
        name="bearing_yoke",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_grey,
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.070),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="tail_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.230),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.050, 0.026, 0.006)),
        origin=Origin(xyz=(0.270, 0.0, -0.003)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0022, length=0.020),
        origin=Origin(xyz=(0.292, 0.0, -0.016)),
        material=white,
        name="stylus",
    )

    model.articulation(
        "arm_height",
        ArticulationType.REVOLUTE,
        parent=pivot,
        child=tonearm,
        origin=Origin(xyz=(0.0, 0.0, 0.040), rpy=(0.0, 0.0, arm_yaw)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.0, lower=-0.04, upper=0.12),
    )

    # Front cueing lever: a small external handle on a hinge barrel at the
    # front edge of the plinth.  Positive motion raises the fingertip.
    cue_lever = model.part("cue_lever")
    cue_lever.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    cue_lever.visual(
        Box((0.016, 0.075, 0.008)),
        origin=Origin(xyz=(0.0, -0.040, 0.007)),
        material=aluminum,
        name="lever_blade",
    )
    cue_lever.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, -0.084, 0.008)),
        material=satin_black,
        name="lever_tip",
    )

    model.articulation(
        "cue_hinge",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=cue_lever,
        origin=Origin(xyz=(0.110, -0.194, 0.096)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.5, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    pivot = object_model.get_part("arm_pivot")
    tonearm = object_model.get_part("tonearm")
    cue_lever = object_model.get_part("cue_lever")

    platter_axle = object_model.get_articulation("platter_axle")
    arm_sweep = object_model.get_articulation("arm_sweep")
    arm_height = object_model.get_articulation("arm_height")
    cue_hinge = object_model.get_articulation("cue_hinge")

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.check(
        "platter uses continuous central axle",
        platter_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={platter_axle.articulation_type}",
    )
    ctx.expect_contact(
        platter,
        plinth,
        elem_a="bearing_hub",
        elem_b="top_plate",
        contact_tol=0.001,
        name="rotating platter hub bears on deck",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="top_plate",
        min_gap=0.002,
        max_gap=0.006,
        name="platter disk clears the top plate",
    )

    ctx.expect_contact(
        pivot,
        plinth,
        elem_a="lower_collar",
        elem_b="arm_pedestal",
        contact_tol=0.001,
        name="tonearm pivot collar is seated on pedestal",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="record_mat",
        min_gap=0.002,
        max_gap=0.008,
        name="stylus hovers just above the rubber mat at rest",
    )

    rest_stylus = elem_center(tonearm, "stylus")
    with ctx.pose({arm_sweep: 0.35}):
        swept_stylus = elem_center(tonearm, "stylus")
    ctx.check(
        "vertical tonearm pivot sweeps stylus across the record area",
        rest_stylus is not None
        and swept_stylus is not None
        and ((swept_stylus[0] - rest_stylus[0]) ** 2 + (swept_stylus[1] - rest_stylus[1]) ** 2) ** 0.5
        > 0.08,
        details=f"rest={rest_stylus}, swept={swept_stylus}",
    )

    with ctx.pose({arm_height: 0.12}):
        raised_stylus = elem_center(tonearm, "stylus")
    ctx.check(
        "horizontal arm-height pivot raises the stylus",
        rest_stylus is not None and raised_stylus is not None and raised_stylus[2] > rest_stylus[2] + 0.020,
        details=f"rest={rest_stylus}, raised={raised_stylus}",
    )

    ctx.expect_contact(
        cue_lever,
        plinth,
        elem_a="hinge_barrel",
        elem_b="cue_saddle",
        contact_tol=0.001,
        name="cueing lever hinge barrel sits on front saddle",
    )
    rest_tip = elem_center(cue_lever, "lever_tip")
    with ctx.pose({cue_hinge: 0.80}):
        raised_tip = elem_center(cue_lever, "lever_tip")
    ctx.check(
        "front cueing lever hinge lifts the fingertip",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.040,
        details=f"rest={rest_tip}, raised={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
