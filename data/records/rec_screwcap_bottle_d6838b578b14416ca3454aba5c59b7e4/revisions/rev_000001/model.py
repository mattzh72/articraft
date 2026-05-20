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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    clear_pet = model.material("clear_pet", rgba=(0.78, 0.88, 0.96, 0.38))
    cap_blue = model.material("cap_blue", rgba=(0.14, 0.36, 0.74, 1.0))
    neck_white = model.material("neck_white", rgba=(0.93, 0.94, 0.95, 1.0))

    bottle = model.part("bottle")

    bottle_outer = [
        (0.0, 0.000),
        (0.014, 0.000),
        (0.031, 0.004),
        (0.034, 0.012),
        (0.035, 0.040),
        (0.033, 0.085),
        (0.031, 0.122),
        (0.032, 0.145),
        (0.027, 0.154),
        (0.021, 0.162),
        (0.016, 0.170),
        (0.0144, 0.176),
        (0.014, 0.184),
        (0.014, 0.190),
    ]
    bottle_inner = [
        (0.0, 0.009),
        (0.011, 0.010),
        (0.029, 0.013),
        (0.031, 0.040),
        (0.029, 0.085),
        (0.027, 0.122),
        (0.028, 0.145),
        (0.0235, 0.153),
        (0.0175, 0.161),
        (0.0124, 0.170),
        (0.0109, 0.176),
        (0.0108, 0.184),
        (0.0108, 0.190),
    ]
    bottle_shell = _mesh(
        "bottle_shell",
        LatheGeometry.from_shell_profiles(
            bottle_outer,
            bottle_inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    bottle.visual(bottle_shell, material=clear_pet, name="bottle_shell")
    bottle.visual(
        Cylinder(radius=0.0164, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.1705)),
        material=neck_white,
        name="support_ring",
    )
    for index, z_pos in enumerate((0.176, 0.180, 0.184)):
        bottle.visual(
            Cylinder(radius=0.0148, length=0.0026),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=neck_white,
            name=f"thread_ring_{index+1}",
        )
    bottle.visual(
        Cylinder(radius=0.0140, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=neck_white,
        name="neck_finish",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.190),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    cap = model.part("cap")
    cap_outer = [
        (0.0186, 0.000),
        (0.0186, 0.021),
        (0.0178, 0.024),
    ]
    cap_inner = [
        (0.0164, 0.001),
        (0.0164, 0.020),
        (0.0154, 0.023),
    ]
    cap_shell = _mesh(
        "cap_shell",
        LatheGeometry.from_shell_profiles(
            cap_outer,
            cap_inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    cap.visual(cap_shell, material=cap_blue, name="cap_shell")
    cap.visual(
        Cylinder(radius=0.0156, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, 0.0227)),
        material=cap_blue,
        name="cap_top",
    )
    for rib_index in range(18):
        angle = rib_index * (math.tau / 18.0)
        cap.visual(
            Box((0.0032, 0.0046, 0.016)),
            origin=Origin(
                xyz=(0.0190 * math.cos(angle), 0.0190 * math.sin(angle), 0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_blue,
            name=f"grip_rib_{rib_index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.024),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_joint = object_model.get_articulation("neck_to_cap")

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=1e-6,
        name="cap stays centered on bottle midline",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        elem_a="cap_shell",
        elem_b="neck_finish",
        min_overlap=0.027,
        name="cap covers the neck footprint",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_top",
        negative_elem="neck_finish",
        min_gap=0.001,
        max_gap=0.006,
        name="cap top sits just above bottle finish",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_joint: 2.2}):
        turned_pos = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=1e-6,
            name="cap remains centered while rotated",
        )
    ctx.check(
        "continuous cap rotation does not translate the cap",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
