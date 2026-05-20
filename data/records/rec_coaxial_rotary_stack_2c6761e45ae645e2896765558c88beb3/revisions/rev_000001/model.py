from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_drum_mesh(name: str, *, inner_radius: float, outer_radius: float, height: float):
    """Lathe a chamfered annular rotary-stage body around the local Z axis."""
    chamfer = min(0.008, height * 0.16)
    groove = min(0.006, (outer_radius - inner_radius) * 0.06)
    z0 = -height * 0.5
    z1 = z0 + chamfer
    z2 = height * 0.5 - chamfer
    z3 = height * 0.5
    profile = [
        (inner_radius, z0),
        (outer_radius - groove, z0),
        (outer_radius, z1),
        (outer_radius, z1 + height * 0.18),
        (outer_radius - groove * 1.6, z1 + height * 0.24),
        (outer_radius - groove * 1.6, z2 - height * 0.24),
        (outer_radius, z2 - height * 0.18),
        (outer_radius, z2),
        (outer_radius - groove, z3),
        (inner_radius, z3),
        (inner_radius, z0),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=96), name)


def _annular_race_mesh(name: str, *, inner_radius: float, outer_radius: float, height: float):
    profile = [
        (inner_radius, -height * 0.5),
        (outer_radius, -height * 0.5),
        (outer_radius, height * 0.5),
        (inner_radius, height * 0.5),
        (inner_radius, -height * 0.5),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=72), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_coaxial_rotary_stack")

    dark_steel = model.material("dark_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.66, 1.0))
    ground_blue = model.material("ground_blue", rgba=(0.12, 0.18, 0.25, 1.0))
    bronze = model.material("bronze_bearing", rgba=(0.72, 0.48, 0.24, 1.0))
    ivory_tick = model.material("ivory_tick", rgba=(0.92, 0.88, 0.76, 1.0))
    stage_red = model.material("stage_red", rgba=(0.70, 0.13, 0.10, 1.0))
    stage_orange = model.material("stage_orange", rgba=(0.85, 0.40, 0.10, 1.0))
    stage_teal = model.material("stage_teal", rgba=(0.08, 0.52, 0.55, 1.0))
    stage_violet = model.material("stage_violet", rgba=(0.38, 0.25, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.43, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=ground_blue,
        name="ground_plinth",
    )
    base.visual(
        Cylinder(radius=0.185, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=dark_steel,
        name="lower_bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.450),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=brushed_steel,
        name="common_shaft",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=dark_steel,
        name="shaft_cap",
    )

    # Stationary inner spacers make the single shaft and the separation between
    # the independently rotating rings legible without colliding with the rings.
    for index, z_center, length in [
        (0, 0.101, 0.010),
        (1, 0.211, 0.013),
        (2, 0.309, 0.014),
        (3, 0.397, 0.014),
        (4, 0.472, 0.012),
    ]:
        base.visual(
            Cylinder(radius=0.064, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=brushed_steel,
            name=f"shaft_spacer_{index}",
        )

    for index, bottom_z in enumerate((0.1075, 0.220, 0.318, 0.405)):
        base.visual(
            Cylinder(radius=0.092, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, bottom_z - 0.003)),
            material=bronze,
            name=f"support_washer_{index}",
        )

    # Four small fixed index ticks on the grounded plinth give the rotary stack
    # a machine-tool feel and provide a stationary reference for the moving lugs.
    for index, yaw in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        base.visual(
            Box((0.095, 0.014, 0.008)),
            origin=Origin(xyz=(0.335, 0.0, 0.063), rpy=(0.0, 0.0, yaw)),
            material=ivory_tick,
            name=f"base_tick_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.43, length=0.52),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
    )

    inner_radius = 0.075
    stage_specs = [
        ("stage_0", 0.340, 0.095, 0.155, stage_red, 18.0),
        ("stage_1", 0.280, 0.080, 0.260, stage_orange, 14.0),
        ("stage_2", 0.220, 0.070, 0.353, stage_teal, 10.0),
        ("stage_3", 0.170, 0.060, 0.435, stage_violet, 7.0),
    ]

    for stage_name, outer_radius, height, z_center, material, effort in stage_specs:
        stage = model.part(stage_name)
        stage.visual(
            _annular_drum_mesh(
                f"{stage_name}_annular_body",
                inner_radius=inner_radius,
                outer_radius=outer_radius,
                height=height,
            ),
            material=material,
            name="stage_body",
        )
        stage.visual(
            _annular_race_mesh(
                f"{stage_name}_top_race",
                inner_radius=inner_radius,
                outer_radius=inner_radius + 0.024,
                height=0.008,
            ),
            origin=Origin(xyz=(0.0, 0.0, height * 0.5 + 0.002)),
            material=bronze,
            name="top_bearing_race",
        )
        marker_len = (outer_radius - inner_radius) * 0.54
        marker_x = inner_radius + 0.016 + marker_len * 0.5
        stage.visual(
            Box((marker_len, 0.023, 0.006)),
            origin=Origin(xyz=(marker_x, 0.0, height * 0.5 + 0.003)),
            material=ivory_tick,
            name="top_index_mark",
        )
        lug_depth = 0.050
        stage.visual(
            Box((lug_depth, 0.043, height * 0.55)),
            origin=Origin(xyz=(outer_radius + lug_depth * 0.5 - 0.018, 0.0, 0.0)),
            material=dark_steel,
            name="rim_drive_lug",
        )
        stage.inertial = Inertial.from_geometry(
            Cylinder(radius=outer_radius, length=height),
            mass=1.2 + outer_radius * 5.0,
        )
        model.articulation(
            f"{stage_name}_rotate",
            ArticulationType.REVOLUTE,
            parent=base,
            child=stage,
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=effort, velocity=3.5, lower=-pi, upper=pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stages = [object_model.get_part(f"stage_{index}") for index in range(4)]
    joints = [object_model.get_articulation(f"stage_{index}_rotate") for index in range(4)]

    for index, joint in enumerate(joints):
        ctx.check(
            f"stage {index} is an independent vertical revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0)
            and joint.parent == "base",
            details=f"joint={joint.name}, type={joint.articulation_type}, axis={joint.axis}, parent={joint.parent}",
        )
        ctx.expect_origin_distance(
            stages[index],
            base,
            axes="xy",
            max_dist=0.001,
            name=f"stage {index} joint lies on the common shaft axis",
        )
        ctx.expect_contact(
            stages[index],
            base,
            elem_a="stage_body",
            elem_b=f"support_washer_{index}",
            contact_tol=0.0005,
            name=f"stage {index} is carried by its fixed thrust washer",
        )

    for lower_index, upper_index in zip(range(3), range(1, 4)):
        ctx.expect_gap(
            stages[upper_index],
            stages[lower_index],
            axis="z",
            min_gap=0.006,
            name=f"stage {upper_index} clears stage {lower_index} vertically",
        )

    ctx.expect_gap(
        stages[0],
        base,
        axis="z",
        min_gap=0.006,
        positive_elem="stage_body",
        negative_elem="lower_bearing_housing",
        name="bottom rotary stage stands above the fixed bearing housing",
    )

    rest_lug_aabb = ctx.part_element_world_aabb(stages[0], elem="rim_drive_lug")
    with ctx.pose({joints[0]: pi / 2.0, joints[2]: -pi / 3.0}):
        turned_lug_aabb = ctx.part_element_world_aabb(stages[0], elem="rim_drive_lug")
        stage_1_position = ctx.part_world_position(stages[1])
        stage_3_position = ctx.part_world_position(stages[3])

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_lug_xy = _aabb_center_xy(rest_lug_aabb)
    turned_lug_xy = _aabb_center_xy(turned_lug_aabb)
    ctx.check(
        "a posed stage carries its rim lug around the common shaft",
        rest_lug_xy is not None
        and turned_lug_xy is not None
        and abs(rest_lug_xy[0] - turned_lug_xy[0]) > 0.20
        and abs(rest_lug_xy[1] - turned_lug_xy[1]) > 0.20,
        details=f"rest_lug_xy={rest_lug_xy}, turned_lug_xy={turned_lug_xy}",
    )
    ctx.check(
        "unposed stages remain coaxial while other stages rotate",
        stage_1_position is not None
        and stage_3_position is not None
        and abs(stage_1_position[0]) < 0.001
        and abs(stage_1_position[1]) < 0.001
        and abs(stage_3_position[0]) < 0.001
        and abs(stage_3_position[1]) < 0.001,
        details=f"stage_1_position={stage_1_position}, stage_3_position={stage_3_position}",
    )

    return ctx.report()


object_model = build_object_model()
