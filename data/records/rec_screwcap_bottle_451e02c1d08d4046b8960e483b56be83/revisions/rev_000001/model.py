from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_pet = model.material("bottle_pet", rgba=(0.76, 0.87, 0.97, 0.58))
    cap_blue = model.material("cap_blue", rgba=(0.17, 0.36, 0.73, 1.0))
    cap_shadow = model.material("cap_shadow", rgba=(0.09, 0.18, 0.39, 1.0))

    body_shell = _save_mesh(
        "bottle_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0090, 0.0000),
                (0.0220, 0.0040),
                (0.0325, 0.0180),
                (0.0338, 0.0500),
                (0.0328, 0.1160),
                (0.0304, 0.1500),
                (0.0258, 0.1770),
                (0.0205, 0.1890),
                (0.0170, 0.1980),
                (0.0152, 0.2050),
                (0.0147, 0.2120),
                (0.0147, 0.2200),
                (0.0153, 0.2220),
            ],
            [
                (0.0000, 0.0045),
                (0.0180, 0.0100),
                (0.0302, 0.0200),
                (0.0316, 0.0500),
                (0.0306, 0.1160),
                (0.0284, 0.1500),
                (0.0241, 0.1770),
                (0.0189, 0.1890),
                (0.0151, 0.1980),
                (0.0133, 0.2050),
                (0.0128, 0.2120),
                (0.0128, 0.2195),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    base_foot_ring = _save_mesh(
        "bottle_base_foot_ring",
        TorusGeometry(radius=0.0268, tube=0.0038, radial_segments=18, tubular_segments=56),
    )
    neck_support_ring = _save_mesh(
        "bottle_neck_support_ring",
        TorusGeometry(radius=0.0168, tube=0.0016, radial_segments=16, tubular_segments=48),
    )
    thread_ridge = _save_mesh(
        "bottle_thread_ridge",
        TorusGeometry(radius=0.0152, tube=0.00075, radial_segments=14, tubular_segments=44),
    )

    cap_shell = _save_mesh(
        "bottle_cap_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0178, 0.0000),
                (0.0178, 0.0208),
                (0.0171, 0.0249),
                (0.0152, 0.0282),
            ],
            [
                (0.0168, 0.0018),
                (0.0168, 0.0232),
                (0.0164, 0.0258),
                (0.0152, 0.0270),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    cap_grip_rib = _save_mesh(
        "cap_grip_rib",
        TorusGeometry(radius=0.0182, tube=0.0009, radial_segments=14, tubular_segments=44),
    )
    cap_top_bead = _save_mesh(
        "cap_top_bead",
        TorusGeometry(radius=0.0156, tube=0.0012, radial_segments=14, tubular_segments=40),
    )

    bottle = model.part("bottle_body")
    bottle.visual(body_shell, material=bottle_pet, name="body_shell")
    bottle.visual(
        base_foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0070)),
        material=bottle_pet,
        name="base_foot_ring",
    )
    bottle.visual(
        neck_support_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.1948)),
        material=bottle_pet,
        name="neck_support_ring",
    )
    bottle.visual(
        thread_ridge,
        origin=Origin(xyz=(0.0, 0.0, 0.2025)),
        material=bottle_pet,
        name="thread_ridge_lower",
    )
    bottle.visual(
        thread_ridge,
        origin=Origin(xyz=(0.0, 0.0, 0.2078)),
        material=bottle_pet,
        name="thread_ridge_middle",
    )
    bottle.visual(
        thread_ridge,
        origin=Origin(xyz=(0.0, 0.0, 0.2131)),
        material=bottle_pet,
        name="thread_ridge_upper",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0340, length=0.2220),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.1110)),
    )

    cap = model.part("screw_cap")
    cap.visual(cap_shell, material=cap_blue, name="cap_shell")
    for rib_z, rib_name in (
        (0.0058, "grip_rib_lower"),
        (0.0129, "grip_rib_middle"),
        (0.0198, "grip_rib_upper"),
    ):
        cap.visual(
            cap_grip_rib,
            origin=Origin(xyz=(0.0, 0.0, rib_z)),
            material=cap_blue,
            name=rib_name,
        )
    cap.visual(
        cap_top_bead,
        origin=Origin(xyz=(0.0, 0.0, 0.0248)),
        material=cap_shadow,
        name="cap_top_bead",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0178, length=0.0282),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.0141)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.1976)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("screw_cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.check(
        "cap uses a continuous neck-axis articulation",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS and cap_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={cap_spin.articulation_type}, axis={cap_spin.axis}",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="neck_support_ring",
        max_gap=0.003,
        max_penetration=0.0,
        name="cap skirt sits just above the bottle support ring",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        elem_a="cap_shell",
        elem_b="thread_ridge_upper",
        min_overlap=0.028,
        name="cap is closely wrapped around the threaded neck",
    )

    with ctx.pose({cap_spin: math.pi / 1.8}):
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            positive_elem="cap_shell",
            negative_elem="neck_support_ring",
            max_gap=0.003,
            max_penetration=0.0,
            name="rotated cap skirt stays seated above the support ring",
        )
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            elem_a="cap_shell",
            elem_b="thread_ridge_upper",
            min_overlap=0.028,
            name="rotated cap stays concentrically wrapped around the neck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
