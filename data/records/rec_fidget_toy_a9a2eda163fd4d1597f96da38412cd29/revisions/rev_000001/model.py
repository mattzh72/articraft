from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (radius * cos((tau * index) / segments), radius * sin((tau * index) / segments))
        for index in range(segments)
    ]


def _gear_outer_profile(
    *,
    tooth_count: int,
    root_radius: float,
    tip_radius: float,
    rise_frac: float = 0.10,
    tip_frac: float = 0.40,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = tau / tooth_count
    for tooth_index in range(tooth_count):
        base = tooth_index * step
        a0 = base
        a1 = base + step * rise_frac
        a2 = a1 + step * tip_frac
        a3 = base + step
        points.extend(
            [
                (root_radius * cos(a0), root_radius * sin(a0)),
                (tip_radius * cos(a1), tip_radius * sin(a1)),
                (tip_radius * cos(a2), tip_radius * sin(a2)),
                (root_radius * cos(a3), root_radius * sin(a3)),
            ]
        )
    return points


def _inner_ratchet_profile(
    *,
    tooth_count: int,
    root_radius: float,
    tip_radius: float,
    drop_frac: float = 0.08,
    dwell_frac: float = 0.30,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = tau / tooth_count
    for tooth_index in range(tooth_count):
        base = tooth_index * step
        a0 = base
        a1 = base + step * drop_frac
        a2 = a1 + step * dwell_frac
        a3 = base + step
        points.extend(
            [
                (root_radius * cos(a0), root_radius * sin(a0)),
                (tip_radius * cos(a1), tip_radius * sin(a1)),
                (tip_radius * cos(a2), tip_radius * sin(a2)),
                (root_radius * cos(a3), root_radius * sin(a3)),
            ]
        )
    return points


def _ring_band_mesh():
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _gear_outer_profile(
                tooth_count=28,
                root_radius=0.0380,
                tip_radius=0.0435,
                rise_frac=0.08,
                tip_frac=0.44,
            ),
            [
                _inner_ratchet_profile(
                    tooth_count=28,
                    root_radius=0.0293,
                    tip_radius=0.0276,
                    drop_frac=0.08,
                    dwell_frac=0.32,
                )
            ],
            height=0.010,
            center=True,
        ),
        "outer_ring_band",
    )


def _hub_sleeve_mesh():
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0115, segments=32),
            [_circle_profile(0.0076, segments=32)],
            height=0.009,
            center=True,
        ),
        "outer_ring_hub",
    )


def _pawl_tip_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            [
                (0.0080, -0.0022),
                (0.0148, 0.0000),
                (0.0082, 0.0022),
            ],
            0.004,
            cap=True,
            closed=True,
        ),
        "pawl_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ratchet_ring_fidget_toy")

    housing_shell = model.material("housing_shell", rgba=(0.17, 0.19, 0.21, 1.0))
    housing_cap = model.material("housing_cap", rgba=(0.24, 0.27, 0.30, 1.0))
    ring_metal = model.material("ring_metal", rgba=(0.73, 0.72, 0.67, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    pawl_steel = model.material("pawl_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.45, 0.48, 0.51, 1.0))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=housing_shell,
        name="housing_base",
    )
    housing.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_cap,
        name="inner_deck",
    )
    housing.visual(
        Cylinder(radius=0.0066, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=spindle_steel,
        name="housing_spindle",
    )
    housing.visual(
        Cylinder(radius=0.0100, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00475)),
        material=spindle_steel,
        name="housing_thrust_washer",
    )
    housing.visual(
        Cylinder(radius=0.0091, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=spindle_steel,
        name="housing_spindle_cap",
    )
    housing.visual(
        Cylinder(radius=0.0036, length=0.004),
        origin=Origin(xyz=(0.018, 0.0, 0.002)),
        material=housing_cap,
        name="pawl_pivot_post",
    )
    housing.visual(
        Box((0.013, 0.006, 0.004)),
        origin=Origin(xyz=(0.0115, 0.0, 0.002)),
        material=housing_cap,
        name="pawl_bridge",
    )
    housing.visual(
        Box((0.012, 0.0020, 0.0030)),
        origin=Origin(xyz=(0.0118, -0.0044, 0.0065), rpy=(0.0, 0.0, 0.18)),
        material=spring_steel,
        name="leaf_spring",
    )
    housing.visual(
        Box((0.004, 0.004, 0.0035)),
        origin=Origin(xyz=(0.0060, -0.0050, 0.00625)),
        material=housing_cap,
        name="spring_anchor",
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        _ring_band_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=ring_metal,
        name="ring_band",
    )
    outer_ring.visual(
        _hub_sleeve_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=ring_metal,
        name="ring_hub",
    )
    spoke_angles = (pi / 2.0, 7.0 * pi / 6.0, 11.0 * pi / 6.0)
    for spoke_index, angle in enumerate(spoke_angles):
        outer_ring.visual(
            Box((0.0185, 0.0042, 0.0050)),
            origin=Origin(
                xyz=(0.0205 * cos(angle), 0.0205 * sin(angle), 0.0105),
                rpy=(0.0, 0.0, angle),
            ),
            material=ring_metal,
            name=f"spoke_{spoke_index}",
        )

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.0031, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=pawl_steel,
        name="pawl_pivot",
    )
    pawl.visual(
        Box((0.0115, 0.0048, 0.0040)),
        origin=Origin(xyz=(0.0062, 0.0, 0.0040)),
        material=pawl_steel,
        name="pawl_body",
    )
    pawl.visual(
        _pawl_tip_mesh(),
        material=pawl_steel,
        name="pawl_tip",
    )
    pawl.visual(
        Box((0.0065, 0.0032, 0.0030)),
        origin=Origin(xyz=(-0.0030, -0.0016, 0.0035)),
        material=pawl_steel,
        name="pawl_tail",
    )
    pawl.visual(
        Cylinder(radius=0.0018, length=0.0032),
        origin=Origin(xyz=(-0.0058, -0.0020, 0.0035)),
        material=pawl_steel,
        name="pawl_tail_pad",
    )

    model.articulation(
        "housing_to_outer_ring",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=outer_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=18.0),
    )
    model.articulation(
        "housing_to_pawl",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pawl,
        origin=Origin(xyz=(0.018, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0, lower=-0.30, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    outer_ring = object_model.get_part("outer_ring")
    pawl = object_model.get_part("pawl")
    ring_spin = object_model.get_articulation("housing_to_outer_ring")
    pawl_joint = object_model.get_articulation("housing_to_pawl")

    ctx.check(
        "outer ring uses vertical continuous spin",
        ring_spin.joint_type == ArticulationType.CONTINUOUS and ring_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={ring_spin.joint_type}, axis={ring_spin.axis}",
    )
    pawl_limits = pawl_joint.motion_limits
    ctx.check(
        "pawl uses small vertical pivot range",
        pawl_joint.joint_type == ArticulationType.REVOLUTE
        and pawl_joint.axis == (0.0, 0.0, 1.0)
        and pawl_limits is not None
        and pawl_limits.lower is not None
        and pawl_limits.upper is not None
        and pawl_limits.lower < 0.0 < pawl_limits.upper
        and pawl_limits.upper - pawl_limits.lower < 0.7,
        details=f"type={pawl_joint.joint_type}, axis={pawl_joint.axis}, limits={pawl_limits}",
    )

    ctx.expect_overlap(
        outer_ring,
        housing,
        axes="xy",
        elem_a="ring_hub",
        elem_b="housing_spindle",
        min_overlap=0.013,
        name="ring hub stays centered on the spindle",
    )
    ctx.expect_gap(
        outer_ring,
        housing,
        axis="z",
        positive_elem="ring_hub",
        negative_elem="housing_thrust_washer",
        min_gap=0.0002,
        max_gap=0.0025,
        name="ring hub rides just above the thrust washer",
    )
    ctx.expect_gap(
        housing,
        outer_ring,
        axis="z",
        positive_elem="housing_spindle_cap",
        negative_elem="ring_hub",
        min_gap=0.0002,
        max_gap=0.0030,
        name="spindle cap retains the ring without rubbing",
    )
    ctx.expect_contact(
        pawl,
        outer_ring,
        elem_a="pawl_tip",
        elem_b="ring_band",
        contact_tol=0.0015,
        name="pawl tip sits close enough to engage the ratchet teeth",
    )

    with ctx.pose({ring_spin: 1.2}):
        ctx.expect_overlap(
            outer_ring,
            housing,
            axes="xy",
            elem_a="ring_hub",
            elem_b="housing_spindle",
            min_overlap=0.013,
            name="ring remains concentric while spinning",
        )
        ctx.expect_gap(
            outer_ring,
            housing,
            axis="z",
            positive_elem="ring_hub",
            negative_elem="housing_thrust_washer",
            min_gap=0.0002,
            max_gap=0.0025,
            name="ring keeps its thrust clearance while spinning",
        )

    def _elem_center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) * 0.5

    def _elem_center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    rest_tip = ctx.part_element_world_aabb(pawl, elem="pawl_tip")
    with ctx.pose({pawl_joint: 0.18}):
        opened_tip = ctx.part_element_world_aabb(pawl, elem="pawl_tip")

    ctx.check(
        "pawl tip can retract away from its rest direction",
        rest_tip is not None
        and opened_tip is not None
        and _elem_center_y(opened_tip) is not None
        and _elem_center_y(rest_tip) is not None
        and _elem_center_y(opened_tip) > _elem_center_y(rest_tip) + 0.0015,
        details=f"rest_tip={rest_tip}, opened_tip={opened_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
