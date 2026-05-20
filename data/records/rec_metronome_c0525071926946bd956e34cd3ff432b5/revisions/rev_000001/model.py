from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _rect_section(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d, z),
        (half_w, -half_d, z),
        (half_w, half_d, z),
        (-half_w, half_d, z),
    ]


def _build_weight_geometry():
    depth = 0.030
    left_cheek = ExtrudeGeometry.centered(
        [
            (-0.036, 0.0),
            (-0.024, -0.028),
            (-0.0065, -0.028),
            (-0.0065, 0.028),
            (-0.024, 0.028),
        ],
        depth,
    ).rotate_x(math.pi / 2.0)
    right_cheek = ExtrudeGeometry.centered(
        [
            (0.036, 0.0),
            (0.024, 0.028),
            (0.0065, 0.028),
            (0.0065, -0.028),
            (0.024, -0.028),
        ],
        depth,
    ).rotate_x(math.pi / 2.0)

    geom = left_cheek.merge(right_cheek)

    for y in (-0.009, 0.009):
        geom.merge(BoxGeometry((0.050, 0.010, 0.012)).translate(0.0, y, 0.021))
        geom.merge(BoxGeometry((0.050, 0.010, 0.012)).translate(0.0, y, -0.021))

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductors_metronome")

    plinth_finish = model.material("plinth_finish", rgba=(0.10, 0.08, 0.07, 1.0))
    wood_finish = model.material("wood_finish", rgba=(0.34, 0.17, 0.08, 1.0))
    wood_shadow = model.material("wood_shadow", rgba=(0.22, 0.10, 0.05, 1.0))
    scale_finish = model.material("scale_finish", rgba=(0.88, 0.82, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.63, 0.22, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.64, 0.50, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.63, 0.66, 1.0))

    body = model.part("body")

    lower_shell = mesh_from_geometry(
        section_loft(
            [
                _rect_section(0.230, 0.170, 0.0),
                _rect_section(0.180, 0.130, 0.230),
            ]
        ),
        "lower_shell",
    )
    upper_shell = mesh_from_geometry(
        section_loft(
            [
                _rect_section(0.170, 0.120, 0.0),
                _rect_section(0.110, 0.075, 0.180),
            ]
        ),
        "upper_shell",
    )

    body.visual(
        Box((0.300, 0.220, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=plinth_finish,
        name="plinth",
    )
    body.visual(
        lower_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=wood_finish,
        name="lower_shell",
    )
    body.visual(
        upper_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        material=wood_finish,
        name="upper_shell",
    )
    body.visual(
        Box((0.100, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=wood_finish,
        name="apex_cap",
    )
    body.visual(
        Box((0.066, 0.015, 0.295)),
        origin=Origin(xyz=(0.0, 0.049, 0.194)),
        material=wood_shadow,
        name="scale_spine",
    )
    body.visual(
        Box((0.040, 0.003, 0.245)),
        origin=Origin(xyz=(0.0, 0.0595, 0.186)),
        material=scale_finish,
        name="scale_plate",
    )
    body.visual(
        Box((0.024, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.040, 0.430)),
        material=wood_shadow,
        name="pivot_block",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.044, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.103, 0.0, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="key_bushing",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.300, 0.220, 0.445)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.2225)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.013, 0.008, 0.340)),
        origin=Origin(xyz=(0.0, 0.032, -0.175)),
        material=brass,
        name="rod_blade",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.024, 0.030, 0.350)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.015, -0.170)),
    )

    weight = model.part("weight")
    weight.visual(
        mesh_from_geometry(_build_weight_geometry(), "slider_weight"),
        material=brass,
        name="slider_weight",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.072, 0.030, 0.056)),
        mass=0.12,
        origin=Origin(),
    )

    key = model.part("key")
    key.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="key_shaft",
    )
    key.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="key_hub",
    )
    key.visual(
        Box((0.020, 0.012, 0.006)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=steel,
        name="key_stem",
    )
    key.visual(
        Box((0.010, 0.038, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=steel,
        name="key_wing",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.066, 0.040, 0.020)),
        mass=0.05,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.053, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.032, -0.105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=0.10,
            lower=0.0,
            upper=0.205,
        ),
    )
    model.articulation(
        "body_to_key",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=key,
        origin=Origin(xyz=(0.106, 0.0, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("key")

    pendulum_joint = object_model.get_articulation("body_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("body_to_key")

    ctx.expect_contact(
        pendulum,
        body,
        elem_a="pivot_hub",
        elem_b="pivot_boss",
        contact_tol=1e-6,
        name="pendulum hub seats on the apex shaft boss",
    )
    ctx.expect_gap(
        pendulum,
        body,
        axis="y",
        min_gap=0.004,
        positive_elem="rod_blade",
        negative_elem="upper_shell",
        name="pendulum rod clears the housing front",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="xy",
        min_overlap=0.008,
        elem_a="slider_weight",
        elem_b="rod_blade",
        name="weight stays guided around the pendulum rod",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    rod_rest = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_blade"))
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper}):
        rod_swung = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_blade"))
    ctx.check(
        "pendulum swings laterally from the apex shaft",
        rod_rest is not None and rod_swung is not None and abs(rod_swung[0] - rod_rest[0]) > 0.070,
        details=f"rest={rod_rest}, swung={rod_swung}",
    )

    weight_rest = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: weight_joint.motion_limits.upper}):
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="xy",
            min_overlap=0.008,
            elem_a="slider_weight",
            elem_b="rod_blade",
            name="weight remains guided on the rod at its low setting",
        )
        weight_low = ctx.part_world_position(weight)
    ctx.check(
        "weight slides downward along the rod as the setting increases",
        weight_rest is not None and weight_low is not None and weight_low[2] < weight_rest[2] - 0.15,
        details=f"rest={weight_rest}, low={weight_low}",
    )

    wing_rest = ctx.part_element_world_aabb(key, elem="key_wing")
    with ctx.pose({key_joint: math.pi / 2.0}):
        wing_quarter = ctx.part_element_world_aabb(key, elem="key_wing")

    rest_dims = None if wing_rest is None else tuple(wing_rest[1][i] - wing_rest[0][i] for i in range(3))
    quarter_dims = None if wing_quarter is None else tuple(wing_quarter[1][i] - wing_quarter[0][i] for i in range(3))
    ctx.check(
        "winding key rotates about its side-face shaft",
        rest_dims is not None
        and quarter_dims is not None
        and rest_dims[1] > rest_dims[2]
        and quarter_dims[2] > quarter_dims[1],
        details=f"rest_dims={rest_dims}, quarter_dims={quarter_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
