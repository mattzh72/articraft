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


BASE_BEAM_X = 0.065
BASE_BEAM_Y = 0.300
BASE_BEAM_Z = 0.032

TREAD_X = 0.055
TREAD_Y = 0.160
TREAD_Z = 0.008
TREAD_OFFSET_X = 0.060

BARREL_OUTER_R = 0.038
BARREL_INNER_R = 0.029
BARREL_COLLAR_R = 0.050
BARREL_HEIGHT = 0.400
TOP_CAP_THICKNESS = 0.014
ROD_GUIDE_R = 0.0115

HANDLE_TRAVEL = 0.220
PISTON_ROD_R = 0.0095
PISTON_ROD_LENGTH = 0.280
PISTON_ROD_CENTER_Z = 0.147
HANDLE_HUB_R = 0.018
HANDLE_HUB_LENGTH = 0.048
HANDLE_HUB_CENTER_Z = 0.031
HANDLE_STEM_R = 0.012
HANDLE_STEM_LENGTH = 0.110
HANDLE_STEM_CENTER_Z = 0.103
HANDLE_BAR_R = 0.014
HANDLE_BAR_LENGTH = 0.300
HANDLE_BAR_CENTER_Z = 0.160
HANDLE_TOP_SEAT_Z = TOP_CAP_THICKNESS / 2.0
GRIP_R = 0.018
GRIP_LENGTH = 0.060
GRIP_OFFSET_X = 0.120

GUIDE_OUTER_X = 0.140
GUIDE_OUTER_Y = 0.028
GUIDE_OUTER_Z = 0.024
GUIDE_INNER_Y = 0.017
GUIDE_INNER_Z = 0.015
GUIDE_CENTER_Y = -0.163
GUIDE_CENTER_Z = 0.014

STABILIZER_SHAFT_X = 0.220
STABILIZER_SHAFT_Y = 0.014
STABILIZER_SHAFT_Z = 0.012
STABILIZER_END_X = 0.040
STABILIZER_END_Y = 0.030
STABILIZER_END_Z = 0.020
STABILIZER_END_OFFSET_X = 0.090
STABILIZER_TRAVEL = 0.050

HANDLE_JOINT_Z = BASE_BEAM_Z + BARREL_HEIGHT - TOP_CAP_THICKNESS / 2.0


def _make_barrel_body():
    sleeve = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_R)
        .extrude(BARREL_HEIGHT)
        .cut(cq.Workplane("XY").circle(BARREL_INNER_R).extrude(BARREL_HEIGHT))
    )
    base_collar = (
        cq.Workplane("XY")
        .circle(BARREL_COLLAR_R)
        .extrude(0.018)
        .cut(cq.Workplane("XY").circle(BARREL_INNER_R).extrude(0.018))
    )
    top_cap = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_R + 0.004)
        .extrude(TOP_CAP_THICKNESS)
        .cut(cq.Workplane("XY").circle(PISTON_ROD_R).extrude(TOP_CAP_THICKNESS))
        .translate((0.0, 0.0, BARREL_HEIGHT - TOP_CAP_THICKNESS))
    )
    rod_guide = (
        cq.Workplane("XY")
        .circle(ROD_GUIDE_R)
        .extrude(0.028)
        .cut(cq.Workplane("XY").circle(PISTON_ROD_R).extrude(0.028))
        .translate((0.0, 0.0, BARREL_HEIGHT - 0.028))
    )
    barrel = sleeve.union(base_collar).union(top_cap).union(rod_guide).combine()
    return barrel.translate((0.0, 0.0, BASE_BEAM_Z)).val()


def _make_lower_guide():
    outer = cq.Workplane("XY").box(
        GUIDE_OUTER_X,
        GUIDE_OUTER_Y,
        GUIDE_OUTER_Z,
        centered=(True, True, True),
    )
    inner = cq.Workplane("XY").box(
        GUIDE_OUTER_X + 0.010,
        GUIDE_INNER_Y,
        GUIDE_INNER_Z,
        centered=(True, True, True),
    )
    return outer.cut(inner).translate((0.0, GUIDE_CENTER_Y, GUIDE_CENTER_Z)).val()


def _make_handle_top():
    collar = (
        cq.Workplane("XY")
        .circle(HANDLE_HUB_R)
        .extrude(HANDLE_HUB_LENGTH)
    )
    stem = (
        cq.Workplane("XY")
        .circle(HANDLE_STEM_R)
        .extrude(HANDLE_STEM_LENGTH)
        .translate((0.0, 0.0, HANDLE_HUB_LENGTH))
    )
    crossbar = (
        cq.Workplane("YZ")
        .circle(HANDLE_BAR_R)
        .extrude(HANDLE_BAR_LENGTH)
        .translate((-HANDLE_BAR_LENGTH / 2.0, 0.0, HANDLE_BAR_CENTER_Z))
    )
    return (
        collar.union(stem)
        .union(crossbar)
        .combine()
        .translate((0.0, 0.0, HANDLE_TOP_SEAT_Z))
        .val()
    )


def _add_rotated_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    axis: str,
    material: str,
) -> None:
    rpy_by_axis = {
        "x": (0.0, 1.5707963267948966, 0.0),
        "y": (1.5707963267948966, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy_by_axis[axis]),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_floor_pump")

    model.material("painted_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("matte_black", rgba=(0.06, 0.06, 0.07, 1.0))

    pump_frame = model.part("pump_frame")
    pump_frame.visual(
        Box((BASE_BEAM_X, BASE_BEAM_Y, BASE_BEAM_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_BEAM_Z / 2.0)),
        material="painted_steel",
        name="base_beam",
    )
    pump_frame.visual(
        Box((TREAD_X, TREAD_Y, TREAD_Z)),
        origin=Origin(
            xyz=(-TREAD_OFFSET_X, 0.0, BASE_BEAM_Z + TREAD_Z / 2.0),
        ),
        material="matte_black",
        name="left_tread",
    )
    pump_frame.visual(
        Box((TREAD_X, TREAD_Y, TREAD_Z)),
        origin=Origin(
            xyz=(TREAD_OFFSET_X, 0.0, BASE_BEAM_Z + TREAD_Z / 2.0),
        ),
        material="matte_black",
        name="right_tread",
    )
    pump_frame.visual(
        mesh_from_cadquery(_make_barrel_body(), "pump_barrel_body"),
        material="brushed_steel",
        name="barrel_body",
    )
    pump_frame.visual(
        mesh_from_cadquery(_make_lower_guide(), "pump_lower_guide"),
        material="painted_steel",
        name="lower_guide",
    )
    pump_frame.inertial = Inertial.from_geometry(
        Box((0.18, 0.30, 0.46)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    handle_assembly = model.part("handle_assembly")
    handle_assembly.visual(
        Cylinder(radius=PISTON_ROD_R, length=PISTON_ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PISTON_ROD_CENTER_Z)),
        material="brushed_steel",
        name="piston_rod",
    )
    handle_assembly.visual(
        mesh_from_cadquery(_make_handle_top(), "pump_handle_top"),
        material="painted_steel",
        name="handle_top",
    )
    _add_rotated_cylinder(
        handle_assembly,
        name="left_grip",
        radius=GRIP_R,
        length=GRIP_LENGTH,
        xyz=(-GRIP_OFFSET_X, 0.0, HANDLE_BAR_CENTER_Z),
        axis="x",
        material="dark_rubber",
    )
    _add_rotated_cylinder(
        handle_assembly,
        name="right_grip",
        radius=GRIP_R,
        length=GRIP_LENGTH,
        xyz=(GRIP_OFFSET_X, 0.0, HANDLE_BAR_CENTER_Z),
        axis="x",
        material="dark_rubber",
    )
    handle_assembly.inertial = Inertial.from_geometry(
        Box((HANDLE_BAR_LENGTH, 0.04, 0.34)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    stabilizer_bar = model.part("stabilizer_bar")
    stabilizer_bar.visual(
        Box((STABILIZER_SHAFT_X, STABILIZER_SHAFT_Y, STABILIZER_SHAFT_Z)),
        material="brushed_steel",
        name="stabilizer_shaft",
    )
    stabilizer_bar.visual(
        Box((STABILIZER_END_X, STABILIZER_END_Y, STABILIZER_END_Z)),
        origin=Origin(xyz=(-STABILIZER_END_OFFSET_X, 0.0, 0.0)),
        material="dark_rubber",
        name="left_stabilizer_pad",
    )
    stabilizer_bar.visual(
        Box((STABILIZER_END_X, STABILIZER_END_Y, STABILIZER_END_Z)),
        origin=Origin(xyz=(STABILIZER_END_OFFSET_X, 0.0, 0.0)),
        material="dark_rubber",
        name="right_stabilizer_pad",
    )
    stabilizer_bar.inertial = Inertial.from_geometry(
        Box((0.22, 0.03, 0.02)),
        mass=0.45,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=pump_frame,
        child=handle_assembly,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HANDLE_TRAVEL,
            effort=180.0,
            velocity=0.50,
        ),
    )
    model.articulation(
        "frame_to_stabilizer",
        ArticulationType.PRISMATIC,
        parent=pump_frame,
        child=stabilizer_bar,
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-STABILIZER_TRAVEL,
            upper=STABILIZER_TRAVEL,
            effort=120.0,
            velocity=0.30,
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

    pump_frame = object_model.get_part("pump_frame")
    handle_assembly = object_model.get_part("handle_assembly")
    stabilizer_bar = object_model.get_part("stabilizer_bar")
    handle_slide = object_model.get_articulation("frame_to_handle")
    stabilizer_slide = object_model.get_articulation("frame_to_stabilizer")

    barrel_body = pump_frame.get_visual("barrel_body")
    lower_guide = pump_frame.get_visual("lower_guide")
    piston_rod = handle_assembly.get_visual("piston_rod")
    handle_top = handle_assembly.get_visual("handle_top")
    stabilizer_shaft = stabilizer_bar.get_visual("stabilizer_shaft")

    ctx.expect_within(
        handle_assembly,
        pump_frame,
        axes="xy",
        inner_elem=piston_rod,
        outer_elem=barrel_body,
        margin=0.003,
        name="piston rod stays centered in barrel",
    )
    ctx.expect_gap(
        handle_assembly,
        pump_frame,
        axis="z",
        positive_elem=piston_rod,
        negative_elem=barrel_body,
        min_gap=0.0,
        max_gap=0.003,
        name="collapsed piston rod starts just above barrel cap",
    )
    ctx.expect_contact(
        handle_assembly,
        pump_frame,
        elem_a=handle_top,
        elem_b=barrel_body,
        contact_tol=0.001,
        name="collapsed handle top seats on barrel cap",
    )

    rest_handle_pos = ctx.part_world_position(handle_assembly)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        ctx.expect_within(
            handle_assembly,
            pump_frame,
            axes="xy",
            inner_elem=piston_rod,
            outer_elem=barrel_body,
            margin=0.003,
            name="extended piston rod stays centered in barrel",
        )
        ctx.expect_gap(
            handle_assembly,
            pump_frame,
            axis="z",
            positive_elem=piston_rod,
            negative_elem=barrel_body,
            min_gap=0.18,
            max_gap=0.23,
            name="extended piston rod lifts above barrel cap",
        )
        extended_handle_pos = ctx.part_world_position(handle_assembly)

    ctx.check(
        "t-handle rises along cylinder axis",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.18,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.expect_within(
        stabilizer_bar,
        pump_frame,
        axes="yz",
        inner_elem=stabilizer_shaft,
        outer_elem=lower_guide,
        margin=0.003,
        name="stabilizer shaft stays nested in lower guide",
    )
    ctx.expect_overlap(
        stabilizer_bar,
        pump_frame,
        axes="x",
        elem_a=stabilizer_shaft,
        elem_b=lower_guide,
        min_overlap=0.12,
        name="centered stabilizer bar remains retained in guide",
    )

    rest_stabilizer_pos = ctx.part_world_position(stabilizer_bar)
    with ctx.pose({stabilizer_slide: STABILIZER_TRAVEL}):
        ctx.expect_within(
            stabilizer_bar,
            pump_frame,
            axes="yz",
            inner_elem=stabilizer_shaft,
            outer_elem=lower_guide,
            margin=0.003,
            name="extended stabilizer shaft stays nested in guide",
        )
        ctx.expect_overlap(
            stabilizer_bar,
            pump_frame,
            axes="x",
            elem_a=stabilizer_shaft,
            elem_b=lower_guide,
            min_overlap=0.12,
            name="extended stabilizer bar still retains insertion",
        )
        extended_stabilizer_pos = ctx.part_world_position(stabilizer_bar)

    ctx.check(
        "stabilizer bar slides across lower frame",
        rest_stabilizer_pos is not None
        and extended_stabilizer_pos is not None
        and extended_stabilizer_pos[0] > rest_stabilizer_pos[0] + 0.04,
        details=f"rest={rest_stabilizer_pos}, extended={extended_stabilizer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
