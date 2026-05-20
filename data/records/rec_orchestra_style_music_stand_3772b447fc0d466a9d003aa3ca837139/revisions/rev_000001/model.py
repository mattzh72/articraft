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
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_stepped_tube_mesh(
    sections: list[tuple[float, float, float]],
    *,
    sides: int = 28,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []

    for z, outer_radius, inner_radius in sections:
        outer_loop: list[int] = []
        inner_loop: list[int] = []
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            c = math.cos(angle)
            s = math.sin(angle)
            outer_loop.append(geom.add_vertex(outer_radius * c, outer_radius * s, z))
            inner_loop.append(geom.add_vertex(inner_radius * c, inner_radius * s, z))
        outer_loops.append(outer_loop)
        inner_loops.append(inner_loop)

    for loop_index in range(len(sections) - 1):
        outer_lower = outer_loops[loop_index]
        outer_upper = outer_loops[loop_index + 1]
        inner_lower = inner_loops[loop_index]
        inner_upper = inner_loops[loop_index + 1]
        for index in range(sides):
            next_index = (index + 1) % sides
            _add_quad(
                geom,
                outer_lower[index],
                outer_lower[next_index],
                outer_upper[next_index],
                outer_upper[index],
            )
            _add_quad(
                geom,
                inner_lower[index],
                inner_upper[index],
                inner_upper[next_index],
                inner_lower[next_index],
            )

    bottom_outer = outer_loops[0]
    bottom_inner = inner_loops[0]
    top_outer = outer_loops[-1]
    top_inner = inner_loops[-1]
    for index in range(sides):
        next_index = (index + 1) % sides
        _add_quad(
            geom,
            bottom_outer[index],
            bottom_inner[index],
            bottom_inner[next_index],
            bottom_outer[next_index],
        )
        _add_quad(
            geom,
            top_outer[index],
            top_outer[next_index],
            top_inner[next_index],
            top_inner[index],
        )

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.25, 0.26, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    lower_sleeve_mesh = mesh_from_geometry(
        _build_stepped_tube_mesh(
            [
                (0.000, 0.0185, 0.0136),
                (0.520, 0.0185, 0.0136),
                (0.521, 0.0280, 0.0139),
                (0.600, 0.0280, 0.0139),
            ],
            sides=32,
        ),
        "music_stand_lower_sleeve",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.185, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=base_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=satin_black,
        name="base_pedestal",
    )
    base.visual(
        Cylinder(radius=0.031, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=satin_black,
        name="base_socket",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="floor_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.09)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        lower_sleeve_mesh,
        material=satin_black,
        name="lower_sleeve",
    )
    lower_mast.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(0.037, 0.0, 0.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="knob_boss",
    )
    lower_mast.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(
            xyz=(0.050, 0.0, 0.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="shaft_stub",
    )
    lower_mast.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.600)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.0115, length=1.050),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=graphite,
        name="upper_tube",
    )
    upper_column.visual(
        Box((0.030, 0.040, 0.052)),
        origin=Origin(xyz=(0.0, -0.008, 0.665)),
        material=satin_black,
        name="head_spine",
    )
    upper_column.visual(
        Box((0.142, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, 0.701)),
        material=satin_black,
        name="support_bridge",
    )
    upper_column.visual(
        Box((0.142, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, -0.024, 0.720)),
        material=satin_black,
        name="hinge_carrier",
    )
    upper_column.visual(
        Box((0.012, 0.018, 0.026)),
        origin=Origin(xyz=(-0.074, -0.014, 0.720)),
        material=satin_black,
        name="left_hinge_ear",
    )
    upper_column.visual(
        Box((0.012, 0.018, 0.026)),
        origin=Origin(xyz=(0.074, -0.014, 0.720)),
        material=satin_black,
        name="right_hinge_ear",
    )
    upper_column.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_black,
        name="stop_collar",
    )
    upper_column.inertial = Inertial.from_geometry(
        Box((0.34, 0.06, 1.08)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.005, 0.185)),
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.0072, length=0.130),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="tray_hinge_barrel",
    )
    tray.visual(
        Box((0.128, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, -0.010)),
        material=graphite,
        name="tray_upper_return",
    )
    tray.visual(
        Box((0.520, 0.004, 0.330)),
        origin=Origin(xyz=(0.0, 0.0, -0.171)),
        material=graphite,
        name="score_panel",
    )
    tray.visual(
        Box((0.014, 0.020, 0.305)),
        origin=Origin(xyz=(-0.253, 0.008, -0.178)),
        material=graphite,
        name="left_flange",
    )
    tray.visual(
        Box((0.014, 0.020, 0.305)),
        origin=Origin(xyz=(0.253, 0.008, -0.178)),
        material=graphite,
        name="right_flange",
    )
    tray.visual(
        Box((0.088, 0.018, 0.235)),
        origin=Origin(xyz=(0.0, -0.006, -0.200)),
        material=satin_black,
        name="center_stiffener",
    )
    tray.visual(
        Box((0.480, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, -0.334)),
        material=graphite,
        name="lower_shelf",
    )
    tray.visual(
        Box((0.480, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.038, -0.324)),
        material=graphite,
        name="lower_lip_front",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.54, 0.06, 0.36)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.010, -0.180)),
    )

    locking_knob = model.part("locking_knob")
    locking_knob.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(
            xyz=(0.008, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="knob_hub",
    )
    locking_knob.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(
            xyz=(0.015, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="knob_wheel",
    )
    locking_knob.visual(
        Box((0.012, 0.010, 0.004)),
        origin=Origin(xyz=(0.016, 0.015, 0.0)),
        material=satin_black,
        name="knob_tab",
    )
    locking_knob.inertial = Inertial.from_geometry(
        Box((0.030, 0.040, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_mast",
        ArticulationType.FIXED,
        parent=base,
        child=lower_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )
    model.articulation(
        "lower_mast_to_upper_column",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.18,
            lower=0.0,
            upper=0.240,
        ),
    )
    model.articulation(
        "upper_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=tray,
        origin=Origin(xyz=(0.0, -0.011, 0.720)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.20,
            upper=0.70,
        ),
    )
    model.articulation(
        "lower_mast_to_locking_knob",
        ArticulationType.CONTINUOUS,
        parent=lower_mast,
        child=locking_knob,
        origin=Origin(xyz=(0.055, 0.0, 0.555)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_mast = object_model.get_part("lower_mast")
    upper_column = object_model.get_part("upper_column")
    tray = object_model.get_part("tray")
    locking_knob = object_model.get_part("locking_knob")

    mast_slide = object_model.get_articulation("lower_mast_to_upper_column")
    tray_tilt = object_model.get_articulation("upper_column_to_tray")
    knob_spin = object_model.get_articulation("lower_mast_to_locking_knob")

    ctx.expect_contact(
        locking_knob,
        lower_mast,
        elem_a="knob_hub",
        elem_b="shaft_stub",
        contact_tol=1e-5,
        name="locking knob seats against the collar shaft",
    )
    ctx.expect_within(
        upper_column,
        lower_mast,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="lower_sleeve",
        margin=0.0025,
        name="upper column stays centered in the lower sleeve at rest",
    )
    ctx.expect_overlap(
        upper_column,
        lower_mast,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_sleeve",
        min_overlap=0.340,
        name="collapsed stand keeps deep mast insertion",
    )
    ctx.expect_gap(
        tray,
        upper_column,
        axis="y",
        positive_elem="tray_upper_return",
        negative_elem="hinge_carrier",
        min_gap=0.003,
        max_gap=0.018,
        name="tray sits just forward of the top bracket",
    )

    rest_upper_pos = ctx.part_world_position(upper_column)
    with ctx.pose({mast_slide: 0.240}):
        ctx.expect_within(
            upper_column,
            lower_mast,
            axes="xy",
            inner_elem="upper_tube",
            outer_elem="lower_sleeve",
            margin=0.0025,
            name="upper column stays centered in the sleeve when extended",
        )
        ctx.expect_overlap(
            upper_column,
            lower_mast,
            axes="z",
            elem_a="upper_tube",
            elem_b="lower_sleeve",
            min_overlap=0.100,
            name="extended stand retains insertion in the sleeve",
        )
        extended_upper_pos = ctx.part_world_position(upper_column)

    ctx.check(
        "upper column extends upward",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.22,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    rest_lip_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="lower_lip_front"))
    with ctx.pose({tray_tilt: 0.55}):
        tilted_lip_center = _aabb_center(
            ctx.part_element_world_aabb(tray, elem="lower_lip_front")
        )
    ctx.check(
        "tray tilt swings the lower lip rearward",
        rest_lip_center is not None
        and tilted_lip_center is not None
        and tilted_lip_center[1] < rest_lip_center[1] - 0.12,
        details=f"rest={rest_lip_center}, tilted={tilted_lip_center}",
    )

    rest_tab_center = _aabb_center(
        ctx.part_element_world_aabb(locking_knob, elem="knob_tab")
    )
    with ctx.pose({knob_spin: 1.2}):
        spun_tab_center = _aabb_center(
            ctx.part_element_world_aabb(locking_knob, elem="knob_tab")
        )
    ctx.check(
        "locking knob actually spins about its shaft",
        rest_tab_center is not None
        and spun_tab_center is not None
        and abs(spun_tab_center[2] - rest_tab_center[2]) > 0.010,
        details=f"rest={rest_tab_center}, spun={spun_tab_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
