from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TAU = 2.0 * math.pi


def _annular_cylinder_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    """A hollow cylindrical ring with real open center, used for storage lips."""

    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(segments):
        a = TAU * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Top annular face.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Bottom annular face.
        geom.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        geom.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])

    return geom


def _blade_airfoil_mesh(
    *,
    length: float = 0.420,
    root_chord: float = 0.108,
    tip_chord: float = 0.074,
    thickness: float = 0.011,
    stations: int = 18,
    chord_samples: int = 9,
) -> MeshGeometry:
    """A tapered, gently cambered retractable fan blade extending along local +X."""

    geom = MeshGeometry()
    top: list[list[int]] = []
    bottom: list[list[int]] = []

    for i in range(stations):
        t = i / (stations - 1)
        # The blade starts just beyond the hinge boss and tapers to a softened tip.
        x = 0.033 + length * t
        tip_rounding = 1.0 - 0.48 * max(0.0, (t - 0.82) / 0.18) ** 2
        chord = (root_chord * (1.0 - t) + tip_chord * t) * tip_rounding
        center_y = 0.026 * math.sin(math.pi * t) - 0.010 * t

        row_top: list[int] = []
        row_bottom: list[int] = []
        for j in range(chord_samples):
            v = -1.0 + 2.0 * j / (chord_samples - 1)
            y = center_y + 0.5 * chord * v
            # A subtle twist and camber keeps the blade from reading as a flat plate.
            mid_z = 0.0045 * math.sin(math.pi * t) * (1.0 - 0.35 * v * v) - 0.004 * t * v
            local_thickness = thickness * (0.72 + 0.28 * (1.0 - abs(v)))
            row_top.append(geom.add_vertex(x, y, mid_z + 0.5 * local_thickness))
            row_bottom.append(geom.add_vertex(x, y, mid_z - 0.5 * local_thickness))
        top.append(row_top)
        bottom.append(row_bottom)

    # Top and bottom skins.
    for i in range(stations - 1):
        for j in range(chord_samples - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    # Leading/trailing edges.
    for i in range(stations - 1):
        for j in (0, chord_samples - 1):
            geom.add_face(top[i][j], bottom[i][j], bottom[i + 1][j])
            geom.add_face(top[i][j], bottom[i + 1][j], top[i + 1][j])

    # Root and tip caps.
    for j in range(chord_samples - 1):
        geom.add_face(top[0][j], top[0][j + 1], bottom[0][j + 1])
        geom.add_face(top[0][j], bottom[0][j + 1], bottom[0][j])
        geom.add_face(top[-1][j + 1], top[-1][j], bottom[-1][j])
        geom.add_face(top[-1][j + 1], bottom[-1][j], bottom[-1][j + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_blade_ceiling_fan")

    satin_white = model.material("satin_white", rgba=(0.86, 0.84, 0.78, 1.0))
    warm_white = model.material("warm_white", rgba=(0.95, 0.92, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    dark_shadow = model.material("shadow_gap", rgba=(0.04, 0.045, 0.05, 1.0))
    maple = model.material("pale_maple", rgba=(0.78, 0.58, 0.34, 1.0))

    housing = model.part("housing")

    # A shallow round motor shell above a broad annular tray.  The underside of the
    # tray is open so the folded blades are visible without colliding with a solid disk.
    motor_shell = LatheGeometry(
        [
            (0.000, 0.000),
            (0.185, 0.000),
            (0.235, 0.036),
            (0.230, 0.116),
            (0.150, 0.175),
            (0.000, 0.175),
        ],
        segments=96,
    )
    housing.visual(
        mesh_from_geometry(motor_shell, "motor_shell"),
        material=satin_white,
        name="motor_shell",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=0.170,
                outer_radius=0.565,
                z_min=-0.062,
                z_max=-0.028,
                segments=112,
            ),
            "storage_ring",
        ),
        material=warm_white,
        name="storage_ring",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=0.535,
                outer_radius=0.565,
                z_min=-0.096,
                z_max=-0.034,
                segments=112,
            ),
            "outer_storage_lip",
        ),
        material=warm_white,
        name="outer_storage_lip",
    )
    housing.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(
                inner_radius=0.145,
                outer_radius=0.192,
                z_min=-0.080,
                z_max=-0.064,
                segments=72,
            ),
            "inner_shadow_gap",
        ),
        material=dark_shadow,
        name="inner_shadow_gap",
    )
    housing.visual(
        Cylinder(radius=0.172, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=satin_white,
        name="lower_motor_collar",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=brushed_steel,
        name="downrod",
    )
    canopy = LatheGeometry(
        [
            (0.000, 0.000),
            (0.082, 0.000),
            (0.100, 0.022),
            (0.088, 0.056),
            (0.025, 0.074),
            (0.000, 0.074),
        ],
        segments=72,
    )
    housing.visual(
        mesh_from_geometry(canopy, "ceiling_canopy"),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=satin_white,
        name="ceiling_canopy",
    )

    rotor = model.part("rotor")
    lower_spinner = LatheGeometry(
        [
            (0.000, -0.056),
            (0.072, -0.050),
            (0.118, -0.024),
            (0.118, 0.022),
            (0.000, 0.022),
        ],
        segments=80,
    )
    rotor.visual(
        mesh_from_geometry(lower_spinner, "lower_spinner"),
        material=brushed_steel,
        name="lower_spinner",
    )
    rotor.visual(
        Cylinder(radius=0.115, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=brushed_steel,
        name="hub_crown",
    )
    rotor.visual(
        Cylinder(radius=0.045, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=brushed_steel,
        name="bearing_shaft",
    )

    rotor_joint = model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    rotor_joint.meta["description"] = "Continuous running rotation of the blade-and-hub assembly."

    blade_mesh = mesh_from_geometry(_blade_airfoil_mesh(), "retractable_blade_airfoil")
    hinge_radius = 0.180
    blade_z_offsets = (0.018, 0.000, -0.018)

    for i, z_offset in enumerate(blade_z_offsets):
        theta = TAU * i / 3.0
        ct = math.cos(theta)
        st = math.sin(theta)

        # Hub arms sit just above each blade root and carry a visible hinge pin.
        arm_mid_r = 0.100
        arm_length = 0.160
        rotor.visual(
            Box((arm_length, 0.030, 0.010)),
            origin=Origin(
                xyz=(arm_mid_r * ct, arm_mid_r * st, z_offset + 0.020),
                rpy=(0.0, 0.0, theta),
            ),
            material=brushed_steel,
            name=f"hinge_arm_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(
                xyz=(hinge_radius * ct, hinge_radius * st, z_offset + 0.006),
            ),
            material=brushed_steel,
            name=f"hinge_pin_{i}",
        )

        blade = model.part(f"blade_{i}")
        blade.visual(
            blade_mesh,
            material=maple,
            name="blade_airfoil",
        )
        blade.visual(
            Cylinder(radius=0.040, length=0.014),
            material=maple,
            name="root_boss",
        )
        blade.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=brushed_steel,
            name="pivot_cap",
        )

        hinge = model.articulation(
            f"rotor_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=blade,
            # At q=0 the blade lies tangentially inside the circular tray; at
            # q=pi/2 it swings out radially for running.
            origin=Origin(
                xyz=(hinge_radius * ct, hinge_radius * st, z_offset),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=3.5,
                lower=0.0,
                upper=math.pi / 2.0,
            ),
        )
        hinge.meta["folded_angle"] = 0.0
        hinge.meta["running_angle"] = math.pi / 2.0

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("housing_to_rotor")

    ctx.check(
        "hub assembly has continuous running rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {spin.articulation_type!r}",
    )
    ctx.expect_contact(
        rotor,
        housing,
        elem_a="bearing_shaft",
        elem_b="lower_motor_collar",
        contact_tol=0.001,
        name="rotating shaft is seated in motor housing bearing",
    )

    for i in range(3):
        blade = object_model.get_part(f"blade_{i}")
        hinge = object_model.get_articulation(f"rotor_to_blade_{i}")
        limits = hinge.motion_limits

        ctx.check(
            f"blade_{i} has a folding revolute hinge",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper > 1.45,
            details=f"type={hinge.articulation_type!r}, limits={limits!r}",
        )

        ctx.allow_overlap(
            rotor,
            blade,
            elem_a=f"hinge_pin_{i}",
            elem_b="root_boss",
            reason="The visible hinge pin is intentionally captured inside the blade root boss.",
        )
        ctx.expect_within(
            rotor,
            blade,
            axes="xy",
            inner_elem=f"hinge_pin_{i}",
            outer_elem="root_boss",
            margin=0.002,
            name=f"blade_{i} hinge pin is centered in root boss",
        )
        ctx.expect_overlap(
            rotor,
            blade,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b="root_boss",
            min_overlap=0.010,
            name=f"blade_{i} hinge pin passes through root boss",
        )
        ctx.allow_overlap(
            rotor,
            blade,
            elem_a=f"hinge_pin_{i}",
            elem_b="pivot_cap",
            reason="The metal pivot cap is drawn seated on the same captured hinge pin.",
        )
        ctx.expect_within(
            rotor,
            blade,
            axes="xy",
            inner_elem=f"hinge_pin_{i}",
            outer_elem="pivot_cap",
            margin=0.002,
            name=f"blade_{i} hinge pin is centered in pivot cap",
        )
        ctx.expect_overlap(
            rotor,
            blade,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b="pivot_cap",
            min_overlap=0.010,
            name=f"blade_{i} pivot cap is seated on hinge pin",
        )

        with ctx.pose({hinge: 0.0}):
            ctx.expect_within(
                blade,
                housing,
                axes="xy",
                inner_elem="blade_airfoil",
                outer_elem="storage_ring",
                margin=0.008,
                name=f"blade_{i} folds within round housing footprint",
            )

    blade_0 = object_model.get_part("blade_0")
    hinge_0 = object_model.get_articulation("rotor_to_blade_0")
    folded_aabb = ctx.part_element_world_aabb(blade_0, elem="blade_airfoil")
    with ctx.pose({hinge_0: math.pi / 2.0}):
        extended_aabb = ctx.part_element_world_aabb(blade_0, elem="blade_airfoil")
    ctx.check(
        "blade_0 swings from stored to radial running position",
        folded_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][0] > folded_aabb[1][0] + 0.36,
        details=f"folded={folded_aabb}, extended={extended_aabb}",
    )

    running_pose = {
        object_model.get_articulation(f"rotor_to_blade_{i}"): math.pi / 2.0
        for i in range(3)
    }
    with ctx.pose(running_pose):
        for i in range(3):
            ctx.expect_gap(
                housing,
                object_model.get_part(f"blade_{i}"),
                axis="z",
                positive_elem="outer_storage_lip",
                negative_elem="blade_airfoil",
                min_gap=0.003,
                name=f"blade_{i} clears housing lip when deployed",
            )

    return ctx.report()


object_model = build_object_model()
