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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_serrated_face(
    part,
    *,
    face_name: str,
    tooth_prefix: str,
    x_span: float,
    z_span: float,
    y_center: float,
    z_center: float,
    face_thickness: float,
    tooth_depth: float,
    tooth_count: int,
    outward_sign: float,
    face_material,
    tooth_material,
) -> None:
    part.visual(
        Box((x_span, face_thickness, z_span)),
        origin=Origin(xyz=(0.0, y_center, z_center)),
        material=face_material,
        name=face_name,
    )
    pitch = x_span / (tooth_count + 1)
    tooth_width = pitch * 0.42
    tooth_y = y_center + outward_sign * (face_thickness * 0.5 + tooth_depth * 0.5 - 0.0001)
    for idx in range(tooth_count):
        part.visual(
            Box((tooth_width, tooth_depth, z_span)),
            origin=Origin(
                xyz=(
                    -x_span * 0.5 + pitch * (idx + 1),
                    tooth_y,
                    z_center,
                )
            ),
            material=tooth_material,
            name=f"{tooth_prefix}_{idx + 1}",
        )


def _add_handwheel(part, *, axis: str, rim_material, steel_material, knob_material) -> None:
    if axis == "x":
        wheel_rpy = (0.0, math.pi * 0.5, 0.0)
        screw_xyz = (0.006, 0.0, 0.0)
        thread_xyz = [(0.002 + 0.003 * idx, 0.0, 0.0) for idx in range(3)]
        spoke_specs = (
            ("spoke_main", Box((0.0024, 0.026, 0.0024)), Origin()),
            ("spoke_diag_a", Box((0.0024, 0.020, 0.0024)), Origin(rpy=(math.radians(52.0), 0.0, 0.0))),
            ("spoke_diag_b", Box((0.0024, 0.020, 0.0024)), Origin(rpy=(math.radians(-52.0), 0.0, 0.0))),
        )
        crank_origin = Origin(xyz=(0.0, 0.0135, 0.0))
        crank_box = Box((0.0026, 0.011, 0.0026))
        knob_origin = Origin(xyz=(0.0, 0.0202, 0.0), rpy=wheel_rpy)
    else:
        wheel_rpy = (math.pi * 0.5, 0.0, 0.0)
        screw_xyz = (0.0, 0.006, 0.0)
        thread_xyz = [(0.0, 0.002 + 0.003 * idx, 0.0) for idx in range(3)]
        spoke_specs = (
            ("spoke_main", Box((0.026, 0.0024, 0.0024)), Origin()),
            ("spoke_diag_a", Box((0.020, 0.0024, 0.0024)), Origin(rpy=(0.0, math.radians(52.0), 0.0))),
            ("spoke_diag_b", Box((0.020, 0.0024, 0.0024)), Origin(rpy=(0.0, math.radians(-52.0), 0.0))),
        )
        crank_origin = Origin(xyz=(0.0135, 0.0, 0.0))
        crank_box = Box((0.011, 0.0026, 0.0026))
        knob_origin = Origin(xyz=(0.0202, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0))

    part.visual(
        Cylinder(radius=0.0165, length=0.0032),
        origin=Origin(rpy=wheel_rpy),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.0052, length=0.008),
        origin=Origin(rpy=wheel_rpy),
        material=steel_material,
        name="hub",
    )
    for spoke_name, geometry, origin in spoke_specs:
        part.visual(geometry, origin=origin, material=steel_material, name=spoke_name)
    part.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=screw_xyz, rpy=wheel_rpy),
        material=steel_material,
        name="screw",
    )
    for idx, xyz in enumerate(thread_xyz, start=1):
        part.visual(
            Cylinder(radius=0.0045, length=0.0012),
            origin=Origin(xyz=xyz, rpy=wheel_rpy),
            material=steel_material,
            name=f"thread_ring_{idx}",
        )
    part.visual(crank_box, origin=crank_origin, material=steel_material, name="crank_arm")
    part.visual(
        Cylinder(radius=0.0024, length=0.007),
        origin=knob_origin,
        material=knob_material,
        name="knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_cross_slide_vise")

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.40, 0.42, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.19, 0.24, 0.26, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    oxide = model.material("oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.340, 0.170, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_cast,
        name="foot",
    )
    base.visual(
        Box((0.292, 0.146, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=cast_iron,
        name="table",
    )
    base.visual(
        Box((0.288, 0.094, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=machined_steel,
        name="dovetail_lower",
    )
    base.visual(
        Box((0.288, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=machined_steel,
        name="dovetail_upper",
    )
    base.visual(
        Box((0.288, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.036, 0.038), rpy=(-math.radians(35.0), 0.0, 0.0)),
        material=machined_steel,
        name="dovetail_left_bevel",
    )
    base.visual(
        Box((0.288, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.036, 0.038), rpy=(math.radians(35.0), 0.0, 0.0)),
        material=machined_steel,
        name="dovetail_right_bevel",
    )
    base.visual(
        Box((0.026, 0.030, 0.022)),
        origin=Origin(xyz=(-0.153, 0.0, 0.031)),
        material=cast_iron,
        name="x_screw_lug",
    )
    base.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(-0.176, 0.0, 0.031), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=machined_steel,
        name="x_screw_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.170, 0.050)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    x_stage = model.part("x_stage")
    x_stage.visual(
        Box((0.228, 0.134, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="bridge",
    )
    x_stage.visual(
        Box((0.224, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.058, -0.009)),
        material=dark_cast,
        name="left_shoe",
    )
    x_stage.visual(
        Box((0.224, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.058, -0.009)),
        material=dark_cast,
        name="right_shoe",
    )
    x_stage.visual(
        Box((0.022, 0.112, 0.006)),
        origin=Origin(xyz=(-0.033, -0.006, 0.017)),
        material=machined_steel,
        name="top_rail_left",
    )
    x_stage.visual(
        Box((0.022, 0.112, 0.006)),
        origin=Origin(xyz=(0.033, -0.006, 0.017)),
        material=machined_steel,
        name="top_rail_right",
    )
    x_stage.visual(
        Box((0.118, 0.020, 0.066)),
        origin=Origin(xyz=(0.0, 0.059, 0.047)),
        material=cast_iron,
        name="fixed_jaw_body",
    )
    _add_serrated_face(
        x_stage,
        face_name="fixed_jaw_face",
        tooth_prefix="fixed_tooth",
        x_span=0.096,
        z_span=0.042,
        y_center=0.0475,
        z_center=0.047,
        face_thickness=0.003,
        tooth_depth=0.0015,
        tooth_count=9,
        outward_sign=-1.0,
        face_material=machined_steel,
        tooth_material=oxide,
    )
    x_stage.visual(
        Box((0.024, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, -0.079, 0.019)),
        material=cast_iron,
        name="y_screw_lug",
    )
    x_stage.visual(
        Cylinder(radius=0.0060, length=0.018),
        origin=Origin(xyz=(0.0, -0.093, 0.019), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="y_screw_boss",
    )
    x_stage.inertial = Inertial.from_geometry(
        Box((0.228, 0.134, 0.072)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    y_jaw = model.part("y_jaw")
    y_jaw.visual(
        Box((0.022, 0.084, 0.006)),
        origin=Origin(xyz=(-0.033, -0.006, 0.003)),
        material=machined_steel,
        name="runner_left",
    )
    y_jaw.visual(
        Box((0.022, 0.084, 0.006)),
        origin=Origin(xyz=(0.033, -0.006, 0.003)),
        material=machined_steel,
        name="runner_right",
    )
    y_jaw.visual(
        Box((0.106, 0.092, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=cast_iron,
        name="carriage",
    )
    y_jaw.visual(
        Box((0.112, 0.018, 0.042)),
        origin=Origin(xyz=(0.0, 0.029, 0.039)),
        material=cast_iron,
        name="moving_jaw_body",
    )
    _add_serrated_face(
        y_jaw,
        face_name="moving_jaw_face",
        tooth_prefix="moving_tooth",
        x_span=0.096,
        z_span=0.042,
        y_center=0.0185,
        z_center=0.039,
        face_thickness=0.003,
        tooth_depth=0.0015,
        tooth_count=9,
        outward_sign=1.0,
        face_material=machined_steel,
        tooth_material=oxide,
    )
    y_jaw.inertial = Inertial.from_geometry(
        Box((0.112, 0.092, 0.058)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    x_handwheel = model.part("x_handwheel")
    _add_handwheel(
        x_handwheel,
        axis="x",
        rim_material=oxide,
        steel_material=machined_steel,
        knob_material=handle_black,
    )
    x_handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.014),
        mass=0.22,
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
    )

    y_handwheel = model.part("y_handwheel")
    _add_handwheel(
        y_handwheel,
        axis="y",
        rim_material=oxide,
        steel_material=machined_steel,
        knob_material=handle_black,
    )
    y_handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.014),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.08, lower=-0.034, upper=0.034),
    )
    model.articulation(
        "x_stage_to_y_jaw",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.06, lower=-0.014, upper=0.010),
    )
    model.articulation(
        "base_to_x_handwheel",
        ArticulationType.FIXED,
        parent=base,
        child=x_handwheel,
        origin=Origin(xyz=(-0.198, 0.0, 0.031)),
    )
    model.articulation(
        "x_stage_to_y_handwheel",
        ArticulationType.FIXED,
        parent=x_stage,
        child=y_handwheel,
        origin=Origin(xyz=(0.0, -0.106, 0.019)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_jaw = object_model.get_part("y_jaw")
    x_handwheel = object_model.get_part("x_handwheel")
    y_handwheel = object_model.get_part("y_handwheel")

    x_slide = object_model.get_articulation("base_to_x_stage")
    y_slide = object_model.get_articulation("x_stage_to_y_jaw")

    table = base.get_visual("table")
    dovetail_lower = base.get_visual("dovetail_lower")
    dovetail_upper = base.get_visual("dovetail_upper")
    x_boss = base.get_visual("x_screw_boss")

    bridge = x_stage.get_visual("bridge")
    left_shoe = x_stage.get_visual("left_shoe")
    right_shoe = x_stage.get_visual("right_shoe")
    top_rail_left = x_stage.get_visual("top_rail_left")
    top_rail_right = x_stage.get_visual("top_rail_right")
    fixed_face = x_stage.get_visual("fixed_jaw_face")
    fixed_tooth = x_stage.get_visual("fixed_tooth_1")
    y_boss = x_stage.get_visual("y_screw_boss")

    runner_left = y_jaw.get_visual("runner_left")
    runner_right = y_jaw.get_visual("runner_right")
    carriage = y_jaw.get_visual("carriage")
    moving_face = y_jaw.get_visual("moving_jaw_face")
    moving_tooth = y_jaw.get_visual("moving_tooth_1")

    x_screw = x_handwheel.get_visual("screw")
    y_screw = y_handwheel.get_visual("screw")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        x_stage,
        base,
        axis="z",
        positive_elem=left_shoe,
        negative_elem=table,
        max_gap=0.0005,
        max_penetration=0.0002,
    )
    ctx.expect_gap(
        x_stage,
        base,
        axis="z",
        positive_elem=right_shoe,
        negative_elem=table,
        max_gap=0.0005,
        max_penetration=0.0002,
    )
    ctx.expect_gap(
        x_stage,
        base,
        axis="z",
        positive_elem=bridge,
        negative_elem=dovetail_upper,
        min_gap=0.0015,
        max_gap=0.0035,
    )
    ctx.expect_gap(
        x_stage,
        base,
        axis="z",
        positive_elem=bridge,
        negative_elem=table,
        min_gap=0.0175,
        max_gap=0.0185,
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=dovetail_upper,
        outer_elem=dovetail_lower,
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=dovetail_lower,
        outer_elem=table,
    )
    ctx.expect_gap(
        y_jaw,
        x_stage,
        axis="z",
        positive_elem=runner_left,
        negative_elem=top_rail_left,
        max_gap=0.0005,
        max_penetration=0.0002,
    )
    ctx.expect_gap(
        y_jaw,
        x_stage,
        axis="z",
        positive_elem=runner_right,
        negative_elem=top_rail_right,
        max_gap=0.0005,
        max_penetration=0.0002,
    )
    ctx.expect_gap(
        y_jaw,
        x_stage,
        axis="z",
        positive_elem=carriage,
        negative_elem=bridge,
        min_gap=0.0115,
        max_gap=0.0125,
    )
    ctx.expect_contact(x_handwheel, base, elem_a=x_screw, elem_b=x_boss)
    ctx.expect_contact(y_handwheel, x_stage, elem_a=y_screw, elem_b=y_boss)
    ctx.expect_overlap(
        y_jaw,
        x_stage,
        axes="xz",
        elem_a=moving_face,
        elem_b=fixed_face,
        min_overlap=0.030,
    )
    ctx.expect_gap(
        x_stage,
        y_jaw,
        axis="y",
        positive_elem=fixed_face,
        negative_elem=moving_face,
        min_gap=0.025,
        max_gap=0.027,
    )
    ctx.expect_overlap(
        x_stage,
        x_stage,
        axes="xz",
        elem_a=fixed_tooth,
        elem_b=fixed_face,
        min_overlap=0.0035,
    )
    ctx.expect_overlap(
        y_jaw,
        y_jaw,
        axes="xz",
        elem_a=moving_tooth,
        elem_b=moving_face,
        min_overlap=0.0035,
    )

    with ctx.pose({y_slide: 0.010}):
        ctx.expect_gap(
            x_stage,
            y_jaw,
            axis="y",
            positive_elem=fixed_face,
            negative_elem=moving_face,
            min_gap=0.015,
            max_gap=0.017,
        )
        ctx.expect_overlap(
            y_jaw,
            x_stage,
            axes="xz",
            elem_a=moving_face,
            elem_b=fixed_face,
            min_overlap=0.030,
        )

    with ctx.pose({y_slide: -0.014}):
        ctx.expect_gap(
            x_stage,
            y_jaw,
            axis="y",
            positive_elem=fixed_face,
            negative_elem=moving_face,
            min_gap=0.039,
            max_gap=0.041,
        )

    with ctx.pose({x_slide: 0.034}):
        ctx.expect_gap(
            x_stage,
            base,
            axis="z",
            positive_elem=left_shoe,
            negative_elem=table,
            max_gap=0.0005,
            max_penetration=0.0002,
        )
        ctx.expect_gap(
            y_jaw,
            x_stage,
            axis="z",
            positive_elem=runner_left,
            negative_elem=top_rail_left,
            max_gap=0.0005,
            max_penetration=0.0002,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
