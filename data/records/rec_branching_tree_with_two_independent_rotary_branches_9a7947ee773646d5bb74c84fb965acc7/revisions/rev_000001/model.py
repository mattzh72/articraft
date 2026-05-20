from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="branching_rotary_tree_study")

    steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.56, 1.0))
    dark_steel = model.material("dark_blasted_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.30, 0.32, 0.34, 1.0))
    black = model.material("black_oxide", rgba=(0.03, 0.035, 0.04, 1.0))
    bronze = model.material("bearing_bronze", rgba=(0.72, 0.53, 0.27, 1.0))
    blue = model.material("tempered_blue_steel", rgba=(0.12, 0.18, 0.24, 1.0))
    cover = model.material("dull_access_cover", rgba=(0.39, 0.42, 0.43, 1.0))

    backbone = model.part("backbone")
    backbone.visual(
        Box((0.68, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )
    backbone.visual(
        Box((0.115, 0.165, 0.640)),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=gunmetal,
        name="vertical_spine",
    )
    backbone.visual(
        Box((0.250, 0.235, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=gunmetal,
        name="central_node",
    )
    backbone.visual(
        Box((0.300, 0.230, 0.040)),
        origin=Origin(xyz=(0.100, 0.0, 0.462)),
        material=gunmetal,
        name="upper_saddle_bridge",
    )
    backbone.visual(
        Box((0.300, 0.230, 0.040)),
        origin=Origin(xyz=(-0.100, 0.0, 0.282)),
        material=gunmetal,
        name="lower_saddle_bridge",
    )

    backbone.visual(
        Box((0.185, 0.012, 0.165)),
        origin=Origin(xyz=(0.0, 0.121, 0.440)),
        material=cover,
        name="front_access_cover",
    )
    backbone.visual(
        Box((0.185, 0.012, 0.165)),
        origin=Origin(xyz=(0.0, -0.121, 0.440)),
        material=cover,
        name="rear_access_cover",
    )

    def add_bolt_grid(
        part,
        *,
        prefix: str,
        center_x: float,
        face_y: float,
        center_z: float,
        x_offsets: tuple[float, ...],
        z_offsets: tuple[float, ...],
        radius: float,
        length: float,
        material: Material,
    ) -> None:
        for ix, dx in enumerate(x_offsets):
            for iz, dz in enumerate(z_offsets):
                part.visual(
                    Cylinder(radius=radius, length=length),
                    origin=Origin(
                        xyz=(center_x + dx, face_y, center_z + dz),
                        rpy=(pi / 2.0, 0.0, 0.0),
                    ),
                    material=material,
                    name=f"{prefix}_bolt_{ix}_{iz}",
                )

    add_bolt_grid(
        backbone,
        prefix="front_cover",
        center_x=0.0,
        face_y=0.129,
        center_z=0.440,
        x_offsets=(-0.067, 0.067),
        z_offsets=(-0.057, 0.057),
        radius=0.010,
        length=0.012,
        material=black,
    )
    add_bolt_grid(
        backbone,
        prefix="rear_cover",
        center_x=0.0,
        face_y=-0.129,
        center_z=0.440,
        x_offsets=(-0.067, 0.067),
        z_offsets=(-0.057, 0.057),
        radius=0.010,
        length=0.012,
        material=black,
    )

    def add_root_carrier(prefix: str, hub_x: float, hub_z: float) -> None:
        for side_name, y in (("front", 0.118), ("rear", -0.118)):
            backbone.visual(
                Box((0.185, 0.032, 0.054)),
                origin=Origin(xyz=(hub_x, y * 0.82, hub_z + 0.061)),
                material=dark_steel,
                name=f"{prefix}_{side_name}_carrier_top",
            )
            backbone.visual(
                Box((0.185, 0.032, 0.054)),
                origin=Origin(xyz=(hub_x, y * 0.82, hub_z - 0.061)),
                material=dark_steel,
                name=f"{prefix}_{side_name}_carrier_bottom",
            )
            backbone.visual(
                Cylinder(radius=0.058, length=0.028),
                origin=Origin(xyz=(hub_x, y, hub_z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=bronze,
                name=f"{prefix}_{side_name}_bearing",
            )
            backbone.visual(
                Box((0.150, 0.012, 0.038)),
                origin=Origin(xyz=(hub_x, y + (0.018 if y > 0.0 else -0.018), hub_z + 0.058)),
                material=cover,
                name=f"{prefix}_{side_name}_cap_upper",
            )
            backbone.visual(
                Box((0.150, 0.012, 0.038)),
                origin=Origin(xyz=(hub_x, y + (0.018 if y > 0.0 else -0.018), hub_z - 0.058)),
                material=cover,
                name=f"{prefix}_{side_name}_cap_lower",
            )
            add_bolt_grid(
                backbone,
                prefix=f"{prefix}_{side_name}_cap",
                center_x=hub_x,
                face_y=y + (0.026 if y > 0.0 else -0.026),
                center_z=hub_z,
                x_offsets=(-0.052, 0.052),
                z_offsets=(-0.058, 0.058),
                radius=0.007,
                length=0.010,
                material=black,
            )
        backbone.visual(
            Box((0.070, 0.200, 0.030)),
            origin=Origin(xyz=(hub_x - (0.070 if hub_x > 0.0 else -0.070), 0.0, hub_z + 0.088)),
            material=black,
            name=f"{prefix}_upper_stop",
        )
        backbone.visual(
            Box((0.070, 0.200, 0.030)),
            origin=Origin(xyz=(hub_x - (0.070 if hub_x > 0.0 else -0.070), 0.0, hub_z - 0.088)),
            material=black,
            name=f"{prefix}_lower_stop",
        )
        backbone.visual(
            Box((0.235, 0.026, 0.026)),
            origin=Origin(xyz=(hub_x * 0.55, 0.092, hub_z - 0.075), rpy=(0.0, -0.32 if hub_x > 0 else 0.32, 0.0)),
            material=steel,
            name=f"{prefix}_front_brace_rib",
        )
        backbone.visual(
            Box((0.235, 0.026, 0.026)),
            origin=Origin(xyz=(hub_x * 0.55, -0.092, hub_z - 0.075), rpy=(0.0, -0.32 if hub_x > 0 else 0.32, 0.0)),
            material=steel,
            name=f"{prefix}_rear_brace_rib",
        )

    add_root_carrier("upper", hub_x=0.200, hub_z=0.560)
    add_root_carrier("lower", hub_x=-0.200, hub_z=0.380)

    def add_branch(name: str, sign: float) -> object:
        branch = model.part(name)
        prefix = "upper" if sign > 0.0 else "lower"
        branch.visual(
            Cylinder(radius=0.025, length=0.270),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{prefix}_shaft",
        )
        branch.visual(
            Cylinder(radius=0.063, length=0.082),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"{prefix}_hub_drum",
        )
        branch.visual(
            Cylinder(radius=0.071, length=0.018),
            origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"{prefix}_front_collar",
        )
        branch.visual(
            Cylinder(radius=0.071, length=0.018),
            origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"{prefix}_rear_collar",
        )
        branch.visual(
            Box((0.475, 0.056, 0.048)),
            origin=Origin(xyz=(sign * 0.280, 0.0, 0.0)),
            material=blue,
            name=f"{prefix}_branch_beam",
        )
        branch.visual(
            Box((0.385, 0.032, 0.018)),
            origin=Origin(xyz=(sign * 0.310, 0.0, 0.033)),
            material=steel,
            name=f"{prefix}_upper_flange",
        )
        branch.visual(
            Box((0.385, 0.032, 0.018)),
            origin=Origin(xyz=(sign * 0.310, 0.0, -0.033)),
            material=steel,
            name=f"{prefix}_lower_flange",
        )
        branch.visual(
            Box((0.330, 0.020, 0.098)),
            origin=Origin(xyz=(sign * 0.275, 0.0, 0.0)),
            material=dark_steel,
            name=f"{prefix}_web_plate",
        )
        branch.visual(
            Box((0.175, 0.024, 0.018)),
            origin=Origin(xyz=(sign * 0.110, 0.018, 0.052), rpy=(0.0, -sign * 0.35, 0.0)),
            material=steel,
            name=f"{prefix}_front_gusset",
        )
        branch.visual(
            Box((0.175, 0.024, 0.018)),
            origin=Origin(xyz=(sign * 0.110, -0.018, 0.052), rpy=(0.0, -sign * 0.35, 0.0)),
            material=steel,
            name=f"{prefix}_rear_gusset",
        )
        branch.visual(
            Box((0.052, 0.076, 0.034)),
            origin=Origin(xyz=(sign * 0.050, 0.0, 0.055)),
            material=black,
            name=f"{prefix}_upper_lug",
        )
        branch.visual(
            Box((0.052, 0.076, 0.034)),
            origin=Origin(xyz=(sign * 0.050, 0.0, -0.055)),
            material=black,
            name=f"{prefix}_lower_lug",
        )
        branch.visual(
            Box((0.105, 0.136, 0.074)),
            origin=Origin(xyz=(sign * 0.565, 0.0, 0.0)),
            material=dark_steel,
            name=f"{prefix}_end_pad",
        )
        for iy, y in enumerate((-0.044, 0.044)):
            for iz, z in enumerate((-0.023, 0.023)):
                branch.visual(
                    Cylinder(radius=0.011, length=0.012),
                    origin=Origin(xyz=(sign * 0.614, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                    material=black,
                    name=f"{prefix}_pad_bolt_{iy}_{iz}",
                )
        return branch

    upper_branch = add_branch("upper_branch", sign=1.0)
    lower_branch = add_branch("lower_branch", sign=-1.0)

    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=upper_branch,
        origin=Origin(xyz=(0.200, 0.0, 0.560)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.45, upper=0.62),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
    )
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=lower_branch,
        origin=Origin(xyz=(-0.200, 0.0, 0.380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.45, upper=0.62),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backbone = object_model.get_part("backbone")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_pivot = object_model.get_articulation("upper_pivot")
    lower_pivot = object_model.get_articulation("lower_pivot")

    for prefix, branch in (("upper", upper_branch), ("lower", lower_branch)):
        for side in ("front", "rear"):
            ctx.allow_overlap(
                backbone,
                branch,
                elem_a=f"{prefix}_{side}_bearing",
                elem_b=f"{prefix}_shaft",
                reason=(
                    "The exposed branch shaft is intentionally captured inside "
                    "the solid bronze bearing-carrier proxy."
                ),
            )
            ctx.expect_within(
                branch,
                backbone,
                axes="xz",
                inner_elem=f"{prefix}_shaft",
                outer_elem=f"{prefix}_{side}_bearing",
                margin=0.0,
                name=f"{prefix} shaft centered in {side} bearing",
            )
            ctx.expect_overlap(
                branch,
                backbone,
                axes="y",
                elem_a=f"{prefix}_shaft",
                elem_b=f"{prefix}_{side}_bearing",
                min_overlap=0.018,
                name=f"{prefix} shaft retained in {side} bearing",
            )

    ctx.expect_origin_distance(
        upper_branch,
        lower_branch,
        axes="xz",
        min_dist=0.40,
        name="pivot origins are visibly separated",
    )
    ctx.check(
        "two independent revolute pivots",
        upper_pivot.articulation_type == ArticulationType.REVOLUTE
        and lower_pivot.articulation_type == ArticulationType.REVOLUTE
        and upper_pivot is not lower_pivot,
        details=f"upper={upper_pivot}, lower={lower_pivot}",
    )

    def element_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    upper_rest_z = element_center_z(upper_branch, "upper_end_pad")
    lower_rest_z = element_center_z(lower_branch, "lower_end_pad")
    with ctx.pose({upper_pivot: 0.50}):
        upper_lifted_z = element_center_z(upper_branch, "upper_end_pad")
        lower_still_z = element_center_z(lower_branch, "lower_end_pad")
    with ctx.pose({lower_pivot: 0.50}):
        lower_lifted_z = element_center_z(lower_branch, "lower_end_pad")
        upper_still_z = element_center_z(upper_branch, "upper_end_pad")

    ctx.check(
        "upper branch rotates independently upward",
        upper_rest_z is not None
        and upper_lifted_z is not None
        and lower_rest_z is not None
        and lower_still_z is not None
        and upper_lifted_z > upper_rest_z + 0.20
        and abs(lower_still_z - lower_rest_z) < 0.005,
        details=(
            f"upper_rest={upper_rest_z}, upper_lifted={upper_lifted_z}, "
            f"lower_rest={lower_rest_z}, lower_during_upper={lower_still_z}"
        ),
    )
    ctx.check(
        "lower branch rotates independently upward",
        lower_rest_z is not None
        and lower_lifted_z is not None
        and upper_rest_z is not None
        and upper_still_z is not None
        and lower_lifted_z > lower_rest_z + 0.20
        and abs(upper_still_z - upper_rest_z) < 0.005,
        details=(
            f"lower_rest={lower_rest_z}, lower_lifted={lower_lifted_z}, "
            f"upper_rest={upper_rest_z}, upper_during_lower={upper_still_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
